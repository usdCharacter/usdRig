#!/usr/bin/env bash
# Smoke-test an extracted Linux bundle on the machine it runs on.
#
#   packaging/vfx/smoke_linux.sh <bundle-dir> [offscreen|xcb|wayland]
#
# offscreen (default): bundle check, Python imports, usdview --help and a
#   viewer launch of both examples with Qt's offscreen platform.
# xcb: the viewer opens both examples on an X server (Xvfb, via
#   xvfb-run) with rendering on, so Storm draws through the host's GL.
# wayland: the same on a headless Weston compositor with Qt's Wayland
#   platform, the windowing path Hyprland (Omarchy) uses.
set -euo pipefail

bundle="$(cd "$1" && pwd)"
mode="${2:-offscreen}"
launcher="$bundle/usdRigView"
examples="$bundle/app/examples"

case "$mode" in
  offscreen)
    "$launcher" --usdRig-check
    (
      while IFS='=' read -r name value; do
        export "$name=$value"
      done < <("$launcher" --usdRig-print-env)
      export QT_QPA_PLATFORM=offscreen
      "$bundle/app/python/bin/python3" -c "
from pxr import Plug, Usd
import rigExecUsdview, shapeEditorUsdview, touchPoseUsdview, usdRigViewMenu
import rigexec, numpy
from UsdNoodles import _usdNoodles
rigExecUsdview._LoadRigExecImaging()
assert Plug.Registry().GetPluginWithName('rigExecUsdview')
s = Usd.Stage.Open('$examples/rigexec_flat.usda')
assert s is not None
a = Usd.Stage.Open('$examples/ArmShotAnim.usda')
assert a and a.GetPrimAtPath('/Shot/HeroArm/Geom/ArmBody')
print('bundle smoke OK')"
      "$bundle/app/python/bin/python3" "$bundle/app/usd/bin/usdview" \
        --help | sed -n '1,5p'
    )
    for stage in rigexec_flat.usda ArmShotAnim.usda; do
      QT_QPA_PLATFORM=offscreen "$launcher" "$examples/$stage" \
        --norender --quitAfterStartup
      echo "offscreen launch OK: $stage"
    done
    ;;
  xcb)
    for stage in rigexec_flat.usda ArmShotAnim.usda; do
      xvfb-run -a -s "-screen 0 1280x1024x24" \
        env QT_QPA_PLATFORM=xcb "$launcher" "$examples/$stage" \
        --quitAfterStartup
      echo "xcb launch OK: $stage"
    done
    ;;
  wayland)
    export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-$(mktemp -d)}"
    chmod 0700 "$XDG_RUNTIME_DIR"
    weston --backend=headless --socket=usdrig-smoke --idle-time=0 \
      > "$XDG_RUNTIME_DIR/weston.log" 2>&1 &
    weston_pid=$!
    trap 'kill "$weston_pid" 2>/dev/null || true' EXIT
    for _ in $(seq 1 50); do
      [ -S "$XDG_RUNTIME_DIR/usdrig-smoke" ] && break
      sleep 0.2
    done
    if [ ! -S "$XDG_RUNTIME_DIR/usdrig-smoke" ]; then
      cat "$XDG_RUNTIME_DIR/weston.log"
      echo "weston did not start" >&2
      exit 1
    fi
    for stage in rigexec_flat.usda ArmShotAnim.usda; do
      WAYLAND_DISPLAY=usdrig-smoke QT_QPA_PLATFORM=wayland \
        LIBGL_ALWAYS_SOFTWARE=1 "$launcher" "$examples/$stage" \
        --quitAfterStartup
      echo "wayland launch OK: $stage"
    done
    ;;
  *)
    echo "unknown mode: $mode" >&2
    exit 2
    ;;
esac
