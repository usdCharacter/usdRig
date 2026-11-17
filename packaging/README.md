# Release bundles

Every `v<major>.<minor>.0` tag builds a self-contained usdRig download for
Windows, macOS (Apple Silicon), and Linux (Ubuntu x86-64) and attaches it
to a GitHub Release: `.zip` on Windows, `.tar.gz` elsewhere, plus a
combined `SHA256SUMS.txt`. A user extracts the archive and runs the
launcher — no OpenUSD, Python, or Qt install is needed and nothing is
written outside the extracted directory.

- Workflow: `.github/workflows/release.yml`
- Launcher: `packaging/launcher/usdRigView.cpp` (built as `usdRigView`)
- Assembly: `packaging/assemble_bundle.py` (stdlib only)
- Tests: `packaging/test_assemble_bundle.py` (ctest:
  `testRigExecBundlePackaging`, needs `-DRIGEXEC_BUILD_VIEWER_LAUNCHER=ON`)
  and `packaging/test_sign_macos.py` (ctest: `testRigExecMacSigning`)

## Bundle layout

Windows/Linux: `usdRigView[.exe]` beside an `app/` tree. macOS:
`usdRigView.app` with the launcher in `Contents/MacOS` and the same
`app/` tree in `Contents/Resources/app`, so the bundle signs and
notarizes as one unit. Inside `app/`:

| Entry | Contents |
|---|---|
| `usd/` | complete OpenUSD install (`bin/usdview`, libraries, `pxr`) |
| `python/` | CPython home OpenUSD was compiled against |
| `rig/` | RigExec `cmake --install` prefix, plus CLI tools in `rig/bin` |
| `plugins/` | `touchPose`, `shapeEditor` usdview plug-ins |
| `examples/` | `rigexec_flat.usda`, `ArmShotAnim.usda`, and its referenced `ArmRig.usda` |
| `icons/` | node icons referenced by the example rig |

The launcher derives all of these paths from its own location, sets
`PYTHONPATH`, `PXR_PLUGINPATH_NAME`, `PATH`, `PYTHONHOME`, and the
platform loader path, then runs `app/usd/bin/usdview` with the bundled
Python. The computed values replace the caller's environment rather
than appending to it (only `PATH` keeps the caller's tail, for child
tools), so another USD or Python install on the user's machine cannot
leak its modules, plug-ins, or DLLs into the bundle. On Windows the
launcher also sets `PXR_USD_WINDOWS_DLL_PATH` to the bundle's library
directories (including `app/rig/bin`, where the node editor's
`noodles.dll` is installed), and the assembler copies the MSVC runtime
(`msvcp140*.dll`, `vcruntime140*.dll`, from the bundled Python's
PySide6 packages) into `app/usd/lib` and `app/rig/lib`, so the bundle
does not depend on the host's Visual C++ runtime version.
`usdRigView --usdRig-check` verifies the bundle without starting the
viewer; `--usdRig-print-env` shows the computed environment. The installed
`rigExecUsdview` directory is also on `PYTHONPATH`, so TouchPose and Shape
Editor can import the shared editor modules, and on `PXR_PLUGINPATH_NAME`,
so usdview loads its plug-in container (the RigExec menu, viewport tools
and pickers).

On macOS, assembly rewrites every Mach-O install name that points at a
build-machine location (a copied CPython framework prefix records
`/Library/Frameworks/Python.framework/...` in the interpreter, libpython,
and every pxr module built against it) to a bundle-relative
`@loader_path` name, and fails if a non-system absolute dependency
remains — such a bundle aborts in dyld on any machine without the build
machine's Python, yet passes a file-existence check on the build machine
itself. `--usdRig-check` parses the same load commands, so a broken
bundle is named before launch instead of dying silently. The only
exceptions are Qt's ODBC/PostgreSQL/Mimer SQL-driver plug-ins, which
usdview never loads. A setup-python `pythonLocation` on macOS is a
symlink shim into a python.org framework install, so assembly bundles
the resolved framework prefix (`Python.framework/Versions/<x.y>`)
instead of the shim tree; copying the shim alone loses
`Resources/Python.app`, which the framework interpreter stub spawns at
run time, and assembly fails if that spawn target is not bundled.
`--trim-dev` removes both `include` and the framework
`Headers` alias; retaining the alias after removing its target breaks
macOS signature verification.

## Cutting a release

1. Bump `project(rigExec VERSION ...)` in `CMakeLists.txt` to match.
   The workflow's gate job refuses a `vX.Y.0` tag that disagrees.
2. Tag and push: `git tag v0.3.0 && git push origin v0.3.0`.
3. The release appears when all three platform jobs finish. Patch tags
   (`v0.3.1`) do not trigger bundles.

`workflow_dispatch` builds the same artifacts without publishing:
give it a `vX.Y.0` version, and set `publish` only when the run should
create/update the Release for that tag.

## Signing

Signing happens on the platform runner, after assembly, before
archiving. On macOS every published release (tag push, or a dispatch
with `publish` set) must sign and notarize: the workflow sets
`RIGEXEC_REQUIRE_NOTARIZATION=1`, so missing credentials fail the job
instead of shipping an unsigned bundle. Unpublished test dispatches
still warn and skip, producing clearly unsigned bundles.

| Platform | Mechanism | Secrets |
|---|---|---|
| Windows | Authenticode (`packaging/sign-windows.ps1`) on every `.exe`/`.dll`/`.pyd`, then the archive is listed in the signed checksums | `WINDOWS_SIGNING_CERT_BASE64`, `WINDOWS_SIGNING_CERT_PASSWORD`, optional `WINDOWS_SIGNING_THUMBPRINT` |
| macOS | `codesign` (hardened runtime) on every Mach-O, then notarization + stapling (`packaging/sign-macos.sh`) | `MACOS_SIGNING_IDENTITY`, `MACOS_CERTIFICATE_BASE64`, `MACOS_CERTIFICATE_PASSWORD`, `AC_API_KEY_ID`, `AC_API_ISSUER_ID`, `AC_API_KEY_BASE64` |
| Linux | No per-`.so` OS signing exists; the publish job GPG-detach-signs `SHA256SUMS.txt` when a key is configured | `LINUX_GPG_PRIVATE_KEY`, `LINUX_GPG_PASSPHRASE` |

Signing orders canonical paths from nested code outward so signing
`Python.app` cannot invalidate the containing framework's resource seal.
The launcher disables Python bytecode writes to preserve that seal during
use. CI verifies the Python library directly as well as the outer app,
including after archive extraction and startup.

A rejected notarization prints Apple's detailed report and stops before
stapling or publishing. To retrieve an older report with the repository's
existing signing credentials, dispatch `notarization-log.yml` with its
`submission_id` input.

## Building a bundle by hand

Given an OpenUSD install built with usdview + Python, a RigExec build,
and that build's Python home:

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DUSD_INSTALL_DIR=$USD -DCMAKE_PREFIX_PATH=$USD \
  -DRIGEXEC_BUILD_VIEWER_LAUNCHER=ON
cmake --build build --target usdRigView rigExecPose rigExecBake rigExecReplayInputs
cmake --install build --prefix rig-install
python3 packaging/assemble_bundle.py \
  --usd-install $USD --rig-install rig-install \
  --python-home /path/to/python-home \
  --launcher build/usdRigView --output dist/usdRig-0.3.0-linux-x64 \
  --version v0.3.0 --platform linux --platform-slug linux-x64 \
  --source-root . --rig-build-dir build --trim-dev \
  --archive dist/usdRig-0.3.0-linux-x64.tar.gz
dist/usdRig-0.3.0-linux-x64/usdRigView --usdRig-check
```

`--trim-dev` drops headers, import/static libraries, and CMake package
files; keep it for release archives (GitHub caps a release asset at
2 GiB) and drop it only for a developer bundle.
