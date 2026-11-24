#!/bin/bash
# Compile the runtime, the EvalCore step table, the op scheduler, the binary
# format, the op core, and the built-in type and op libraries with no OpenUSD
# include path. Fails if a translation unit needs a pxr header, or if an
# undefined symbol is an OpenUSD type.
#
# Usage: bin/check_runtime_usd_free.sh [BUILD_DIR | --sdk MANIFEST_DIR]
#        [--list-objects prints one line per distinct object target]
# With a build directory (default: this checkout's build/) the manifests and
# exact-FP flags come from that configured build tree, exactly as CMake
# generates them, and entries keep the spellings CMake wrote. With --sdk,
# the argument is an installed SDK's manifest directory M and the
# installed-manifest grammar applies: entries whose first path component
# is runtime-generated resolve under M, every other relative entry under
# this checkout (never both), and an absolute or '..' entry marks a
# producer-dependent layout that cannot be checked from a consumer.
# The op core and the type and op libraries come from opCoreSources.txt,
# which an installed SDK ships beside the other manifests; a build tree adds
# libs/rigExecBuiltins/builtins.cpp.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
usage() {
    echo "usage: ${0##*/} [--list-objects] [BUILD_DIR | --sdk MANIFEST_DIR]" >&2
    exit 2
}
BUILD=""
SDK=""
list_objects=0
_rest=()
for _arg in "$@"; do
    if [[ "$_arg" == "--list-objects" ]]; then
        list_objects=1
    else
        _rest+=("$_arg")
    fi
done
set -- "${_rest[@]+"${_rest[@]}"}"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --sdk)
            [[ $# -eq 2 && -n "$2" && -z "$SDK" && -z "$BUILD" ]] || usage
            SDK="$2"; shift 2 ;;
        --build)
            [[ $# -eq 2 && -n "$2" && -z "$BUILD" && -z "$SDK" ]] || usage
            BUILD="$2"; shift 2 ;;
        -h|--help)
            echo "usage: ${0##*/} [--list-objects] [BUILD_DIR | --sdk MANIFEST_DIR]" >&2
            exit 0 ;;
        *)
            [[ $# -eq 1 && -n "$1" && -z "$BUILD" && -z "$SDK" ]] || usage
            BUILD="$1"; shift ;;
    esac
done
BUILD="${BUILD:-$ROOT/build}"
installed=0
if [[ -n "$SDK" ]]; then
    installed=1
    [[ "$SDK" = /* ]] || SDK="$PWD/$SDK"
    SDK="${SDK%/}"
    MANIFESTS="$SDK"
else
    [[ "$BUILD" = /* ]] || BUILD="$PWD/$BUILD"
    BUILD="${BUILD%/}"
    MANIFESTS="$BUILD"
fi
if [[ ! -f "$MANIFESTS/runtimeSources.txt" || ! -f "$MANIFESTS/runtimeIncludes.txt" ||
      ! -f "$MANIFESTS/evalCoreSources.txt" || ! -f "$MANIFESTS/evalCoreIncludes.txt" ||
      ! -f "$MANIFESTS/opCoreSources.txt" ]]; then
    if [[ $installed -eq 1 ]]; then
        echo "no SDK manifests in $MANIFESTS: expected runtimeSources.txt, runtimeIncludes.txt, evalCoreSources.txt, evalCoreIncludes.txt and opCoreSources.txt there (install the SDK from a matching build)" >&2
    else
        echo "configure and build rigExecRuntime first (missing source manifest in $MANIFESTS)" >&2
    fi
    exit 1
fi
if [[ ! -s "$MANIFESTS/exactFpFlags.txt" ]]; then
    echo "configure first: $MANIFESTS/exactFpFlags.txt names the exact-FP flags" >&2
    exit 1
fi
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

# cmake/rigexec_flags.cmake writes this list at configure time, so the flags
# here cannot drift from the ones the build compiles these sources with.
fp_flags=()
while IFS= read -r flag; do
    flag="${flag%$'\r'}"
    [[ -z "$flag" ]] && continue
    fp_flags+=("$flag")
done < "$MANIFESTS/exactFpFlags.txt"
if [[ ${#fp_flags[@]} -eq 0 ]]; then
    echo "no exact-FP flags in $MANIFESTS/exactFpFlags.txt" >&2
    exit 1
fi
for flag in "${fp_flags[@]}"; do
    if [[ "$flag" == *fast-math* && "$flag" != -fno-fast-math ]]; then
        echo "cmake/rigexec_flags.cmake must not name fast math: $flag" >&2
        exit 1
    fi
done

# The exact-FP list is the language-neutral floating-point contract from
# cmake/rigexec_flags.cmake (/fp:precise, or -fno-fast-math -ffp-contract=off),
# so it goes to the C units as well; everything else stays per language.
CXXFLAGS=(
    -std=c++17 -O2 "${fp_flags[@]}" -pthread
    -I "$ROOT/libs"
    -I "$ROOT/thirdparty/flatbuffers/include"
    -I "$ROOT/thirdparty/lzma/C"
)
CFLAGS=(
    -std=c11 -O2 "${fp_flags[@]}" -DZ7_ST -DRIGEXEC_LZMA_PORTABLE_SCALAR
    -I "$ROOT/thirdparty/lzma/C"
)
resolve_manifest_entry() {
    # Prints the path an installed-manifest entry ($2) resolves under: the
    # reserved runtime-generated component joins the manifest directory, a
    # bare relative entry joins this checkout, and never both. Absolute
    # entries and '..' segments are producer-dependent spellings.
    local manifest="$1" entry="$2" root
    case "$entry" in
        /*|..|../*|*/..|*/../*)
            echo "$manifest: $entry: installed entries are relative without '..' segments (this manifest came from a producer build tree: install the SDK from a matching build)" >&2
            return 1
            ;;
    esac
    case "$entry" in
        runtime-generated|runtime-generated/*) root="$SDK" ;;
        *) root="$ROOT" ;;
    esac
    printf '%s' "$root/$entry"
}

for manifest in runtimeIncludes.txt evalCoreIncludes.txt; do
    while IFS= read -r include_dir; do
        include_dir="${include_dir%$'\r'}"
        [[ -z "$include_dir" ]] && continue
        spelling="$include_dir"
        if [[ $installed -eq 1 ]]; then
            include_dir="$(resolve_manifest_entry "$manifest" "$include_dir")" || exit 1
            if [[ ! -d "$include_dir" ]]; then
                echo "$manifest: $spelling: no such include directory $include_dir" >&2
                exit 1
            fi
        else
            [[ "$include_dir" = /* ]] || include_dir="$ROOT/$include_dir"
        fi
        if [[ -d "$include_dir/pxr" ]]; then
            echo "runtime/EvalCore include directories expose OpenUSD: $include_dir" >&2
            exit 1
        fi
        # The include manifest is a compile input for every unit the source
        # manifests name, C included, not a C++-only setting.
        CXXFLAGS+=(-I "$include_dir")
        CFLAGS+=(-I "$include_dir")
    done < "$MANIFESTS/$manifest"
done

objs=()
declare -A object_of=()  # object key -> the source that claimed it
# Each manifest entry is a translation unit: C for .c, C++ for the other final
# suffixes cmake/exportManifests.cmake accepts. The list is case-sensitive here,
# in the exporter, and in the Godot example's SConstruct. Anything else is a
# producer bug and stops the check rather than being skipped.
compile_one() {
    local src="$1" key obj extension
    extension="${src##*.}"
    # The object key carries the whole manifest spelling with '/' folded, so
    # mesh.cpp, a mesh.cxx beside it and a mesh.scalar.cxx each get their own
    # object, and one unit named by two manifests compiles once.
    key="${src#"$ROOT"/}"
    if [[ -n "$SDK" ]]; then
        key="${key#"$SDK"/}"
    fi
    key="${key//\//@}"
    obj="$WORK/$key.o"
    if [[ -n "${object_of[$key]+set}" ]]; then
        if [[ "${object_of[$key]}" != "$src" ]]; then
            echo "${object_of[$key]} and $src would share object $obj: manifest sources must stay distinct" >&2
            exit 1
        fi
        return 0
    fi
    object_of["$key"]="$src"
    case "$extension" in
        c)
            gcc "${CFLAGS[@]}" -c "$src" -o "$obj" ;;
        cc|cxx|cpp|c++)
            g++ "${CXXFLAGS[@]}" -c "$src" -o "$obj" ;;
        *)
            echo "$src: not a translation unit (supported: .c .cc .cxx .cpp .c++)" >&2
            exit 1 ;;
    esac
    objs+=("$obj")
}

sources=()
for manifest in runtimeSources.txt evalCoreSources.txt opCoreSources.txt; do
    while IFS= read -r src; do
        src="${src%$'\r'}"
        [[ -z "$src" ]] && continue
        spelling="$src"
        if [[ $installed -eq 1 ]]; then
            # The installed grammar constrains every entry: resolve and
            # validate before compilation.
            src="$(resolve_manifest_entry "$manifest" "$src")" || exit 1
            if [[ ! -f "$src" ]]; then
                echo "$manifest: $spelling: no such source $src" >&2
                exit 1
            fi
        else
            [[ "$src" = /* ]] || src="$ROOT/$src"
        fi
        sources+=("$src")
    done < "$MANIFESTS/$manifest"
done
if [[ $installed -eq 0 ]]; then
    sources+=("$ROOT/libs/rigExecBuiltins/builtins.cpp")
fi
sources+=(
    "$ROOT/libs/rigExecGraph/opGraph.cpp"
    "$ROOT/libs/rigExecBinary/format.cpp"
    "$ROOT/libs/rigExecBinary/manifest.cpp"
    "$ROOT/libs/rigExecBinary/transport.cpp"
    "$ROOT/thirdparty/lzma/C/LzmaEnc.c"
    "$ROOT/thirdparty/lzma/C/LzmaDec.c"
    "$ROOT/thirdparty/lzma/C/LzFind.c"
    "$ROOT/thirdparty/lzma/C/CpuArch.c"
)

for src in "${sources[@]}"; do
    compile_one "$src"
done

# Symbol lines only: with several objects nm also prints each object's
# path, and a mktemp directory name such as tmp.VtYcwFeMAb matches.
if nm -uC "${objs[@]}" | grep -E '^[[:space:]]+U ' |
    grep -E 'pxr::|[^A-Za-z0-9_](Gf|Usd|Sdf|Vt|Tf|Hd|Exec)[A-Za-z0-9_]*' ; then
    echo "runtime link line names an OpenUSD symbol" >&2
    exit 1
fi

if [[ $list_objects -eq 1 ]]; then
    for key in "${!object_of[@]}"; do
        printf 'object: %s.o\n' "$key"
    done | sort
fi

echo "runtime compiled with no pxr include path (${#objs[@]} objects)"
