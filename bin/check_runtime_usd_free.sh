#!/bin/bash
# Compile the runtime, the op scheduler, and the binary format with no
# OpenUSD include path. Fails if a translation unit needs a pxr header,
# or if an undefined symbol is an OpenUSD type.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="${1:-$ROOT/build}"
if [[ ! -f "$BUILD/runtimeSources.txt" || ! -f "$BUILD/runtimeIncludes.txt" ]]; then
    echo "configure and build rigExecRuntime first (missing source manifest in $BUILD)" >&2
    exit 1
fi
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

CXXFLAGS=(
    -std=c++17 -O2 -fno-fast-math -ffp-contract=off -pthread
    -I "$ROOT/libs"
    -I "$ROOT/thirdparty/flatbuffers/include"
    -I "$ROOT/thirdparty/lzma/C"
)
CFLAGS=(
    -std=c11 -O2 -DZ7_ST -DRIGEXEC_LZMA_PORTABLE_SCALAR
    -I "$ROOT/thirdparty/lzma/C"
)
while IFS= read -r include_dir; do
    include_dir="${include_dir%$'\r'}"
    if [[ -d "$include_dir/pxr" ]]; then
        echo "runtime include directories expose OpenUSD: $include_dir" >&2
        exit 1
    fi
    [[ -z "$include_dir" ]] || CXXFLAGS+=(-I "$include_dir")
done < "$BUILD/runtimeIncludes.txt"

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
while IFS= read -r src; do
    src="${src%$'\r'}"
    [[ -z "$src" ]] && continue
    [[ "$src" = *.cpp || "$src" = *.c ]] || continue
    [[ "$src" = /* ]] || src="$ROOT/$src"
    sources+=("$src")
done < "$BUILD/runtimeSources.txt"
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
