#!/bin/bash
# Compile the runtime, the EvalCore step table, the op scheduler, and the
# binary format with no OpenUSD include path. Fails if a translation unit
# needs a pxr header, or if an undefined symbol is an OpenUSD type.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="${1:-$ROOT/build}"
if [[ ! -f "$BUILD/runtimeSources.txt" || ! -f "$BUILD/runtimeIncludes.txt" ||
      ! -f "$BUILD/evalCoreSources.txt" || ! -f "$BUILD/evalCoreIncludes.txt" ]]; then
    echo "configure and build rigExecRuntime first (missing source manifest in $BUILD)" >&2
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
for manifest in runtimeIncludes.txt evalCoreIncludes.txt; do
    while IFS= read -r include_dir; do
        include_dir="${include_dir%$'\r'}"
        [[ -z "$include_dir" ]] && continue
        [[ "$include_dir" = /* ]] || include_dir="$ROOT/$include_dir"
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
        [[ "$src" = /* ]] || src="$ROOT/$src"
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
