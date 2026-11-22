#!/usr/bin/env bash
# Build and install packaging/aur/PKGBUILD from a local CY2027 Linux bundle,
# then run the installed usdRigView's self-check. Run as root on Arch Linux.
#
#   packaging/aur/test_pkgbuild.sh <usdRig-*-vfx2027-linux-x86_64.tar.gz>
set -euo pipefail

tarball="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"
here="$(cd "$(dirname "$0")" && pwd)"
name="$(basename "$tarball" .tar.gz)"
version="${name#usdRig-}"
version="${version%-vfx2027-linux-x86_64}"

id builder >/dev/null 2>&1 || useradd -m builder
work="$(mktemp -d)"
cp "$here/PKGBUILD" "$tarball" "$work/"
sed -i "s/^pkgver=.*/pkgver=${version}/" "$work/PKGBUILD"
sed -i "s|^source=.*|source=(\"${name}.tar.gz\")|" "$work/PKGBUILD"
chown -R builder "$work"
(cd "$work" && sudo -u builder makepkg -f --noconfirm --nodeps)
pacman -U --noconfirm "$work"/usdrig-bin-*.pkg.tar.*
pacman -Qi usdrig-bin | sed -n '1,4p'
usdRigView --usdRig-check
echo "AUR recipe install OK"
