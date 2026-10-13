---
title: Getting started
---

# Getting started

RigExec builds against an **unmodified OpenUSD 26.08 installation with
OpenExec** and loads into `usdview` as a set of plugins. This page takes you
from a checkout to a character playing in the viewport.

## What you need

| | |
|---|---|
| OpenUSD | **26.08**, built with OpenExec and `usdview` ([upstream build instructions](https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/BUILDING.md)) |
| Build tools | CMake **3.26+**, Ninja, a C++17 compiler |
| Python | The same version and architecture your OpenUSD was built with |

The helper scripts look for a sibling `../usd-install` by default. Set `USD`
to point at another OpenUSD prefix and `PY` at its Python interpreter;
`VENV` optionally selects a virtual environment.

## Build

=== "Windows"

    ```bat
    set USD=C:\path\to\usd-install
    set PY=C:\path\to\python.exe
    bin\build_rigexec.bat
    ```

    The helper finds Visual Studio on its own; `RIGEXEC_VCVARS` overrides the
    toolchain script it uses.

=== "Linux and macOS"

    ```sh
    export USD=/path/to/usd-install
    export PY=/path/to/python
    bin/build_rigexec.sh
    ```

The helper configures, builds and runs the enabled CTest suites. To configure
by hand instead:

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DUSD_INSTALL_DIR=/path/to/usd-install -DCMAKE_PREFIX_PATH=/path/to/usd-install
cmake --build build
ctest --test-dir build --output-on-failure
cmake --install build --prefix /path/to/rigexec-install
```

The installed CMake package exposes targets such as `rigExec::rigExec`:

```cmake
find_package(rigExec CONFIG REQUIRED)
target_link_libraries(myApplication PRIVATE rigExec::rigExec)
```

## Open a character

=== "Windows"

    ```bat
    bin\launch_usdview.bat examples\biped\Biped_stack.usda
    ```

=== "Linux and macOS"

    ```sh
    bin/usdview.sh examples/biped/Biped_stack.usda
    ```

The plugins activate on their own for any stage carrying a `RigExecRoot` and
re-evaluate on every timeline change. Everything RigExec adds to `usdview`
lives under the **RigExec** menu.

<div class="grid cards" markdown>

-   **Pose it**

    ---

    Select a control in the viewport and drag the gizmo, or open
    **RigExec ▸ Animation Editors ▸ Control Picker**.

    [:octicons-arrow-right-24: Animator tools](../tools/index.md)

-   **Play it**

    ---

    Scrub or play the timeline. Frames warm in the background and play from
    the cache.

    [:octicons-arrow-right-24: What warming does](../concepts/frame-cache-warming.md)

-   **See the rig**

    ---

    Turn on *Display ▸ Display Purposes ▸ Guide* to draw joints and solvers
    as guide geometry.

    [:octicons-arrow-right-24: Control guides](../specs/control-guides.md)

</div>

## Dynamic or baked

RigExec evaluates a frame in one of two ways: **dynamic**, through OpenExec,
which follows every authored change; or **baked**, a compiled step program
built for playback. Both produce the same pose. Choose the starting mode with
an environment variable before launching:

```bat
set RIGEXEC_EVALUATION_MODE=baked
```

[Baked and dynamic evaluation](../concepts/baked-vs-dynamic.md) explains
what each is for and how to switch at run time.

## Run the tests

| Suite | Command |
|---|---|
| C++ and Python bindings | run by the build helper, or `ctest --test-dir build` |
| Viewer models (no display) | `bin/test/run_python_tests.sh` or `bin\test\run_python_tests.bat` |
| `usdview` integration | the `bin/test/run_testusdview_*` helpers (need a graphics context) |

## Next

- [Examples](examples.md): the stages that ship with the repository.
- [How operators fire](../concepts/how-operators-fire.md): the mental model
  behind every rig.
- [Node reference](../index.md): every node, its wiring and parameters.
