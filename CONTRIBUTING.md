# Contributing to usdRig

Thanks for interest in usdRig! usdRig is an OpenUSD character-rigging engine
(RigExec) under active development. This guide gets you from fork to merged
pull request.

## Before you start

- Read [AGENTS.md](AGENTS.md) and the [domain glossary](CONTEXT.md) for project
  terminology (`usdRig` = project, `RigExec` = engine).
- Check [STATUS.md](STATUS.md) for which layers are prototype vs. maturing.
- Look at open issues; ones labeled `good first issue` are scoped for
  first-time contributors.

## Environment

Building from source requires OpenUSD 26.08 with OpenExec and `usdview`,
CMake 3.26+, Ninja, and a C++17 compiler, matching the Python version and
architecture of your OpenUSD build. See the README's
[Build and run](README.md#build-and-run) section; the `bin/` helpers handle
configuration for you.

## Making a change

1. Fork the repository and create a branch per change (`fix/...`,
   `feat/...`, `docs/...`).
2. Keep commits atomic: one coherent change per commit, with a succinct
   message describing it (see [AGENTS.md](AGENTS.md#git)).
3. Build and test. `bin/build_rigexec.sh` (Windows: `bin\build_rigexec.bat`)
   configures, builds everything and runs the whole suite; it is the right
   command for a fresh checkout. For an incremental change, run the set it
   affects:
   ```sh
   bin/test_changed.sh --run <changed paths>   # direct set, then the transitive set before pushing
   ctest --test-dir build -L fast              # the short tier
   ```
   Source `bin/_env.sh` (Windows: `bin\_env.bat`) first: `ctest` and anything
   importing `pxr` or `rigexec` needs this build's USD, Python and plugin path.
   Python interaction-math tests: `bin/test/run_python_tests.sh` (or the
   `.bat` equivalent). Viewer integration tests need a working graphics
   context.
4. A new tracked file must be classified in
   [tools/license_manifest.json](tools/license_manifest.json): the license
   audit fails a file no family claims. Run `python3 tools/check_licenses.py`
   after editing the manifest.
5. Format C++ with the pinned CI policy:
   ```sh
   just format        # or: python3 bin/format_cpp.py --fix <files>
   ```
6. For docs changes, preview the site with
   `python -m mkdocs serve` (deps in `docs/requirements.txt`).

## Pull requests

- One PR per coherent change; describe what changed and how it was verified.
- CI (`usdRig` workflow) must pass: it builds and tests the C++ libraries on
  Ubuntu 24.04, Windows 2022 and macOS 15, and checks formatting, hidden
  locks, allocations and the license manifest.
- CI does not run the Python bindings or the `usdview` helpers; run those
  yourself when a change touches them.
- Node documentation pages are generated: edit `docs/operator_notes.py` or
  the schema docs and regenerate with `python docs/build_pages.py`.
- A maintainer reviews and lands the change once CI and review are green on
  the current head; a stale branch is rebased onto main and re-validated.

## Reporting bugs

Use the [bug report template](.github/ISSUE_TEMPLATE/bug_report.md). Include
the USD stage (or a minimal repro), the rig being evaluated, platform, and
the observed vs. expected pose. `rigExecPose` output and CTest diagnostics
are very helpful.
