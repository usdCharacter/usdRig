#!/bin/bash
# Compile the runtime, the EvalCore step table, the op scheduler, and the
# binary format with no OpenUSD include path. Fails if a translation unit
# needs a pxr header, or if an undefined symbol is an OpenUSD type.
#
# Usage: bin/check_runtime_usd_free.sh [manifest-directory]
# Without an argument the manifests and exact-FP flags come from this
# checkout's configured build tree, exactly as CMake generates them. With
# one, the argument is an installed SDK's manifest directory M and the
# installed-manifest grammar applies: entries whose first path component
# is runtime-generated resolve under M, every other relative entry under
# this checkout (never both), and an absolute or '..' entry marks a
# producer-dependent layout that cannot be checked from a consumer.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -gt 1 ]]; then
    echo "usage: ${0##*/} [manifest-directory]" >&2
    exit 2
fi
BUILD="${1:-$ROOT/build}"
installed=0
if [[ $# -eq 1 && -n "${1:-}" ]]; then
    installed=1
    [[ "$BUILD" = /* ]] || BUILD="$PWD/$BUILD"
    BUILD="${BUILD%/}"
fi
if [[ ! -f "$BUILD/runtimeSources.txt" || ! -f "$BUILD/runtimeIncludes.txt" ||
      ! -f "$BUILD/evalCoreSources.txt" || ! -f "$BUILD/evalCoreIncludes.txt" ]]; then
    if [[ $installed -eq 1 ]]; then
        echo "no SDK manifests in $BUILD: expected runtimeSources.txt, runtimeIncludes.txt, evalCoreSources.txt and evalCoreIncludes.txt there (install the SDK from a matching build)" >&2
    else
        echo "configure and build rigExecRuntime first (missing source manifest in $BUILD)" >&2
    fi
    exit 1
fi
if [[ ! -s "$BUILD/exactFpFlags.txt" ]]; then
    echo "configure first: $BUILD/exactFpFlags.txt names the exact-FP flags" >&2
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
done < "$BUILD/exactFpFlags.txt"
if [[ ${#fp_flags[@]} -eq 0 ]]; then
    echo "no exact-FP flags in $BUILD/exactFpFlags.txt" >&2
    exit 1
fi
for flag in "${fp_flags[@]}"; do
    if [[ "$flag" == *fast-math* && "$flag" != -fno-fast-math ]]; then
        echo "cmake/rigexec_flags.cmake must not name fast math: $flag" >&2
        exit 1
    fi
done

CXXFLAGS=(
    -std=c++17 -O2 "${fp_flags[@]}" -pthread
    -I "$ROOT/libs"
    -I "$ROOT/thirdparty/flatbuffers/include"
    -I "$ROOT/thirdparty/lzma/C"
)
CFLAGS=(
    -std=c11 -O2 -DZ7_ST -DRIGEXEC_LZMA_PORTABLE_SCALAR
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
        runtime-generated|runtime-generated/*) root="$BUILD" ;;
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
        CXXFLAGS+=(-I "$include_dir")
    done < "$BUILD/$manifest"
done

objs=()
compile_one() {
    local src="$1"
    local obj="$WORK/${#objs[@]}_$(basename "${src%.*}").o"
    if [[ "$src" == *.c ]]; then
        gcc "${CFLAGS[@]}" -c "$src" -o "$obj"
    else
        g++ "${CXXFLAGS[@]}" -c "$src" -o "$obj"
    fi
    objs+=("$obj")
}

sources=()
for manifest in runtimeSources.txt evalCoreSources.txt; do
    while IFS= read -r src; do
        src="${src%$'\r'}"
        [[ -z "$src" ]] && continue
        [[ "$src" = *.cpp || "$src" = *.c ]] || continue
        spelling="$src"
        if [[ $installed -eq 1 ]]; then
            src="$(resolve_manifest_entry "$manifest" "$src")" || exit 1
            if [[ ! -f "$src" ]]; then
                echo "$manifest: $spelling: no such source $src" >&2
                exit 1
            fi
        else
            [[ "$src" = /* ]] || src="$ROOT/$src"
        fi
        sources+=("$src")
    done < "$BUILD/$manifest"
done
sources+=(
    "$ROOT/libs/rigExecGraph/opGraph.cpp"
    "$ROOT/libs/rigExecBinary/format.cpp"
    "$ROOT/libs/rigExecBinary/transport.cpp"
    "$ROOT/thirdparty/lzma/C/LzmaEnc.c"
    "$ROOT/thirdparty/lzma/C/LzmaDec.c"
    "$ROOT/thirdparty/lzma/C/LzFind.c"
    "$ROOT/thirdparty/lzma/C/CpuArch.c"
)

for src in "${sources[@]}"; do
    compile_one "$src"
done

if nm -uC "${objs[@]}" | grep -E 'pxr::|[^A-Za-z0-9_](Gf|Usd|Sdf|Vt|Tf|Hd|Exec)[A-Za-z0-9_]*' ; then
    echo "runtime link line names an OpenUSD symbol" >&2
    exit 1
fi

echo "runtime compiled with no pxr include path (${#objs[@]} objects)"
