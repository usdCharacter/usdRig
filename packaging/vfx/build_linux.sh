#!/usr/bin/env bash
# Build one VFX Reference Platform year's Linux bundle inside that year's
# ASWF ci-base image (Rocky Linux with the year's gcc and glibc), so every
# binary loads on the year's oldest supported distribution.
#
#   packaging/vfx/build_linux.sh <year> <version> <git-sha>
#
# Run from the repository root, which must also hold an OpenUSD checkout
# in ./OpenUSD. Reuses ./usd-install when it already holds usdview (a
# restored cache). Writes dist/usdRig-<version>-vfx<year>-linux-x86_64
# (+ .tar.gz) and dist/usdRig-sdk-<version>-vfx<year>-linux-x86_64.tar.gz.
#
# Python is a relocatable python-build-standalone CPython (installed with
# uv) rather than the image's /usr/local Python, so the bundle can carry
# its own interpreter without the rest of /usr/local.
set -euo pipefail

year="$1"
version="$2"
git_sha="${3:-}"
jobs="${JOBS:-4}"
root="$(pwd)"

# The image's PYTHONPATH points at its own packages; keep them out of the
# interpreter this build bundles.
export PYTHONPATH=
unset PYTHONHOME

eval "$(python3 packaging/vfx/vfx_profile.py outputs "$year" | sed 's/^/P_/')"
slug="vfx${P_year}-linux-x86_64"

echo "::group::toolchain"
sed -n '1,3p' /etc/os-release
gcc --version | sed -n 1p
ldd --version | sed -n 1p
echo "::endgroup::"

echo "::group::python ${P_python}"
python3 -m pip install --quiet --disable-pip-version-check uv
python3 -m uv python install "${P_python}" --install-dir "$root/pyroot"
pyhome="$(ls -d "$root"/pyroot/cpython-"${P_python}".*-linux-x86_64-gnu | sed -n 1p)"
py="$pyhome/bin/python3"
export PATH="$pyhome/bin:$PATH"
"$py" -m pip install --quiet --disable-pip-version-check \
  --break-system-packages \
  "cmake>=3.27,<4" ninja pybind11 PyOpenGL \
  "PySide6==${P_pyside6}" "numpy==${P_numpy}"
"$py" --version
"$py" -c "import PySide6, numpy; print('PySide6', PySide6.__version__, 'numpy', numpy.__version__)"
echo "::endgroup::"

if [ ! -x usd-install/bin/usdview ]; then
  echo "::group::OpenUSD ${P_openusd} (oneTBB ${P_onetbb_tag}, OpenSubdiv ${P_opensubdiv_tag})"
  "$py" packaging/vfx/vfx_profile.py pin-build-usd "$year" \
    OpenUSD/build_scripts/build_usd.py
  "$py" OpenUSD/build_scripts/build_usd.py usd-install \
    --build usd-build --generator Ninja --build-variant release \
    --jobs "$jobs" --onetbb --usd-imaging --usdview \
    --no-tests --no-examples --no-tutorials --no-docs --no-materialx \
    --build-args USD,"-DPXR_BUILD_EXEC=ON" -v
  rm -rf usd-build
  echo "::endgroup::"
fi
test -f usd-install/bin/usdview

echo "::group::usdRig (C++${P_cxx_standard})"
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DUSD_INSTALL_DIR="$root/usd-install" \
  -DCMAKE_PREFIX_PATH="$root/usd-install" \
  -DPython3_EXECUTABLE="$py" \
  -DCMAKE_CXX_STANDARD="${P_cxx_standard}" \
  -DRIGEXEC_BUILD_PYTHON=ON -DRIGEXEC_BUILD_USDNOODLES=ON \
  -DRIGEXEC_BUILD_TESTS=OFF -DRIGEXEC_BUILD_VIEWER_LAUNCHER=ON
cmake --build build --parallel "$jobs"
rm -rf usdrig-install
cmake --install build --prefix "$root/usdrig-install"
echo "::endgroup::"

echo "::group::bundle ${slug}"
bundle="dist/usdRig-${version}-${slug}"
rm -rf "$bundle"
"$py" packaging/assemble_bundle.py \
  --usd-install usd-install --rig-install usdrig-install \
  --python-home "$pyhome" \
  --launcher build/usdRigView \
  --output "$bundle" --version "$version" \
  --platform linux --platform-slug "$slug" \
  --source-root . --rig-build-dir build \
  --usdnoodles-dir build/python/UsdNoodles \
  --git-sha "$git_sha" --trim-dev
"$py" packaging/vfx/check_bundle.py "$bundle" --platform linux \
  --glibc-max "${P_linux_glibc_max}" \
  --glibcxx-max "${P_linux_glibcxx_max}" --collect
echo "::endgroup::"

echo "::group::smoke test in the build image"
bash packaging/vfx/smoke_linux.sh "$bundle"
echo "::endgroup::"

tar -C dist -czf "$bundle.tar.gz" "$(basename "$bundle")"
tar -C "$root" -czf "dist/usdRig-sdk-${version}-${slug}.tar.gz" \
  --transform "s,^usdrig-install,usdRig-sdk-${version}-${slug}," \
  usdrig-install
ls -la dist
