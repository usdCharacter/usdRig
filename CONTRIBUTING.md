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
3. Build and test:
   ```sh
   bin/build_rigexec.sh              # Windows: bin\build_rigexec.bat
   bin/test_changed.sh --run
   ```
   Python interaction-math tests: `bin/test/run_python_tests.sh` (or the
   `.bat` equivalent). Viewer integration tests need a working graphics
   context.
4. Format C++ with the pinned CI policy:
   ```sh
   just format        # or: python3 bin/format_cpp.py --fix <files>
   ```
5. For docs changes, preview the site with
   `python -m mkdocs serve` (deps in `docs/requirements.txt`).

## Pull requests

- One PR per coherent change; describe what changed and how it was verified.
- CI (`usdRig` workflow) must pass: it builds and tests the C++ libraries on
  Ubuntu 24.04, Windows 2022 and macOS 15, and checks style, hidden locks,
   and allocation rules.
- A maintainer reviews and merges; squash merges are preferred.

## Reporting bugs

Use the [bug report template](.github/ISSUE_TEMPLATE/bug_report.md). Include
the USD stage (or a minimal repro), the rig being evaluated, platform, and
the observed vs. expected pose. `rigExecPose` output and CTest diagnostics
are very helpful.
