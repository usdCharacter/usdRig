# Running the usdNoodles scenario suite

`plugin/usdNoodles/testenv` is the regression net for the node-graph editor. It
is **one CTest test**, `testUsdNoodles`, that runs

```sh
python -m unittest discover -s . -p "test*.py"
```

in the *staged* package directory `build/python/UsdNoodles/testenv`. The
`usdNoodles_python_package` target copies the sources there, and the tests
import `UsdNoodles` and the native `_usdNoodles` module out of the build tree —
never out of `plugin/`. CMake registers the test only when
`RIGEXEC_BUILD_USDNOODLES`, `RIGEXEC_BUILD_TESTS` and Python are all on, and
prepends `build/python` and `build/python/UsdNoodles/testenv` to that test's
`PYTHONPATH` (root `CMakeLists.txt`, the `add_test(NAME testUsdNoodles ...)`
block).

`.github/workflows/usdrig.yml` runs it in the `usdnoodles-suite` job. Its
prerequisite `openusd-py` job builds an OpenUSD v26.08 prefix **with** Python
bindings and usdview; the C++ jobs (`configure`, `build-release`, `ctest-cpp`,
`install-check`, `runtime-usd-free`, `platform-cpp`) keep consuming the
`openusd` prefix, which has neither, so they stay independent of this suite.
This file is the local twin of `usdnoodles-suite`.

## Bring the environment up

Create an interpreter environment outside the repository. It must be the same
CPython minor version the OpenUSD prefix was built against, because the pxr
extension modules are ABI-tagged.

```sh
python3 -m venv "$HOME/venvs/usdNoodles"
"$HOME/venvs/usdNoodles/bin/python3" -m pip install PySide6 PyOpenGL pybind11 numpy
export USD=/path/to/OpenUSD-26.08            # has bin/usdview, bin/testusdview,
                                             # lib/python3.12/site-packages/pxr
export PY="$HOME/venvs/usdNoodles/bin/python3"
. bin/_env.sh
cmake -S . -B build-noodles -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DUSD_INSTALL_DIR="$USD" -DCMAKE_PREFIX_PATH="$USD" \
  -DPython3_EXECUTABLE="$PY" \
  -DRIGEXEC_BUILD_USDNOODLES=ON -DRIGEXEC_BUILD_TESTS=ON -DRIGEXEC_BUILD_PYTHON=ON
cmake --build build-noodles --target usdNoodlesLib _usdNoodles usdNoodles_python_package
ctest --test-dir build-noodles -R '^testUsdNoodles$' --output-on-failure --no-tests=error
```

Two parts of that are load-bearing and easy to lose:

- `-DPython3_EXECUTABLE="$PY"`: CTest runs the suite with **CMake's** `Python3`,
  not with the `$PY` that `bin/_env.sh` resolves. Left to autodetection on a
  Debian-family machine, CMake picks the system interpreter, which has no
  PySide6, and every scenario that imports `UsdNoodles.graphView` fails at
  import.
- `pybind11` in that interpreter: `-DRIGEXEC_BUILD_PYTHON=ON` configures the
  `_rigexec` bindings, and configure stops with "pybind11 not found" without it.
  `numpy` is not used by the scenario suite; it is needed by the editor drivers
  below (`usdview` fills its GL buffers through numpy).

`noodles` itself is not a prerequisite: with no `include/noodles` +
`lib/libnoodles` under `NOODLES_ROOT` (which defaults to `$USD`),
`plugin/usdNoodles/CMakeLists.txt` fetches the SHA-pinned archive over HTTPS and
builds it and GLEW (`GLEW_NO_GLU`) side by side. Point `NOODLES_ROOT` at a
prefix that has them to build offline.

If the tree is anywhere other than `build/`, export `RIGEXEC_BUILD_DIR` (a
relative value is taken from the checkout root) before sourcing `bin/_env.sh`;
that is how the helper scripts in `bin/` find the configured tree:

```sh
export RIGEXEC_BUILD_DIR=build-noodles
```

## Run one file, or one case, directly

```sh
cd build-noodles/python/UsdNoodles/testenv
"$PY" -m unittest -v testUsdNoodlesStageValidation
"$PY" -m unittest -v testUsdNoodlesStageValidation.<Class>.<case>
```

Keep `bin/_env.sh` sourced: `sitecustomize.py` beside the tests reads
`RIGEXEC_USD_INSTALL` (which `_env.sh` exports from `$USD`) to point the test
interpreter at this USD install and at this build tree's plugins.

## The change selector does not reach this suite

`bin/test_changed.sh` and `tools/select_affected_tests.py` prune `plugin/`, so
they report zero executables, zero tests and no libraries for any edit under
`plugin/usdNoodles/`:

```
$ bin/test_changed.sh plugin/usdNoodles/graphView.py
direct tests (0):
transitive tests (0):
```

Run the suite **by name** — the `ctest -R '^testUsdNoodles$'` line above — after
any change to `plugin/usdNoodles/`, and say so in the report; a clean
`test_changed.sh` output proves nothing here.

## Skips are not passes

`plugin/usdNoodles/testenv/headless/base.py` turns a missing Qt/OpenGL stack
into `unittest.SkipTest` for its class, and other files skip explicitly for
platform-specific behaviour. A machine with no GL context therefore reports
`OK (skipped=NN)` while exercising nothing of the GL path. Always report the
collected case count **and** the pass/fail/skip counts, never just "passed":
the skip count is the number that changes when an environment goes away, and a
falling collected-case count means discovery stopped early.

## Editor drivers (viewport paths, outside the suite)

`bin/test/run_testusdview_noodles_{rename,values,icons,touch}.sh` drive real Qt
events through `testusdview` against the tracked fixtures
`tests/fixtures/noodles_{rename,values,icons}.usda` and
`examples/biped/Biped_stack.usda`, and each prints its own marker
(`RIGEXEC_NOODLES_RENAME_OK`, `RIGEXEC_NOODLES_VALUES_OK`,
`RIGEXEC_NOODLES_ICONS_OK`, `RIGEXEC_NOODLES_TOUCH_OK`). `usdnoodles-suite` runs
all four after the suite and fails when a marker is missing.

```sh
bash bin/test/run_testusdview_noodles_rename.sh
```

- The four `.sh` drivers are checked in **without** the executable bit, so run
  them with `bash` (the `.bat` twins are the Windows entry point).
- `GraphView` derives from `pxr.Usdviewq.qt.QGLWidget`, which usdview maps to
  `QOpenGLWidget` under PySide6, and the Qt offscreen platform does not support
  that widget at all ("QOpenGLWidget is not supported on this
  platform"). These drivers need a GLX display: a desktop session, or
  `xvfb-run -a --server-args="-screen 0 1920x1080x24"` in front of each with
  `libgl1-mesa-dev xorg-dev` installed, which is what CI does. A display where
  the GL context cannot be created (`QOpenGLWidget: Failed to create context`)
  never runs `initializeGL`, and then `rename`, `values` and `icons` fail with
  `AttributeError: 'GraphView' object has no attribute '_cppIconRenderer'`, the
  renderer `initializeGL` creates. That signature appears on the unmodified
  tree too, so it marks an unusable display, not a broken editor.
- They need `build-noodles/python/UsdNoodles/plugInfo.json` (staged by
  `usdNoodles_python_package`), `numpy`, and this build tree's
  `librigExecImaging.so`: build `--target rigExecImaging` as well, or
  `testusdview` exits non-zero on its Tf error stream — *after* printing the
  marker, because `PXR_PLUGINPATH_NAME` from `bin/_env.sh` names the plugin.

## Baseline on the unmodified tree

Measured 2026-11-19 at `3a4e487` on Ubuntu 24.04.5 / Linux aarch64, GCC 13.3,
an OpenUSD 26.08 prefix built with Python 3.12 and usdview, a venv with
PySide6 6.12.0 / PyOpenGL 3.1.10 / pybind11 3.1.0 / numpy 2.5.5, and noodles
fetched by FetchContent at `ff5d473f10e8`:

```
$ ctest --test-dir build-noodles -R '^testUsdNoodles$' --output-on-failure --no-tests=error
1/1 Test #549: testUsdNoodles ......... Passed  18.3 sec
Ran 1289 tests in 17.982s
OK (skipped=46)
```

1289 cases collected, 1243 passed, 0 failed, 0 errors, 46 skipped. The skips
break down as:

| count | reason |
|---|---|
| 28 | `Headless harness not available` — the four `testUsdNoodlesHeadless*.py` files import the harness as `from tests.headless.base import ...`, and no package named `tests` is importable in this layout (the directory is `testenv`): `ModuleNotFoundError: No module named 'tests'`. They skip on every platform, GL present or not, so **no case in this suite exercises the GL render path.** The same files import cleanly as `headless.base`. |
| 16 | `macOS-only` |
| 2 | skipped by name for a known process-kill teardown (`NodeModel` holding a live USD prim during GC) |

Later tasks compare against **1289 collected / 46 skipped / 0 failed / 0
errors**; a count that moves needs an explanation in the change that moved it.
The stage-sync files raise it to **1333 collected / 47 skipped / 0 failed / 0
errors** (1286 passed) on this host (measured 2026-11-20): 44 added cases in
`testUsdNoodlesStageSync.py`, 42 for stage replacement and reload and 2 for
dock-window close. The one skip above the 46 baseline is
`RigExecFkChain schema not registered` in `testUsdNoodlesValueModel.py`, which
depends on `PXR_PLUGINPATH_NAME` reaching the rigExecSchema resources; it is
present with and without the dock cases, so it is an environment difference,
not a change in the suite.

The editor drivers on the same tree:

- `run_testusdview_noodles_rename.sh`, `..._icons.sh`, `..._touch.sh`: each
  printed its marker and exited 0, run against a GLX display that reported a
  GL 4.1 context, with `librigExecImaging.so` built.
- On the current host (aarch64, no GLX display; Qt reports
  `QOpenGLWidget is not supported on this platform`) `..._touch.sh` prints
  `RIGEXEC_NOODLES_TOUCH_OK` and exits 0, while `..._rename.sh`, `..._values.sh`
  and `..._icons.sh` exit 1 with
  `AttributeError: 'GraphView' object has no attribute '_cppIconRenderer'`.
  The same three signatures appear on the tree before these changes, so they
  mark an unusable display rather than the editor.
- `run_testusdview_noodles_values.sh`: **not verified on that host.** The
  command `bash bin/test/run_testusdview_noodles_values.sh` exits 1 on the
  driver's own precondition —
  `AssertionError: Noodles Editor opened outside the available screens:
  PySide6.QtCore.QRect(0, -37, 402, 374)` — because that X server reports a
  zero-size screen to Qt (`app.screens() → QRect(0, 0, 0, 0)`), so no window can
  satisfy the assertion. That is an unusable display, not a passing or failing
  editor. The offscreen Qt platform cannot substitute for a GLX display here
  (`QOpenGLWidget is not supported on this platform`), so the values driver
  stays unverified until the `usdnoodles-suite` run on the PR head reports it.
