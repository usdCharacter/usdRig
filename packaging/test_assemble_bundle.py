#!/usr/bin/env python3
"""Synthetic-tree tests for packaging/assemble_bundle.py.

Builds fake OpenUSD / RigExec / CPython trees in a temp directory, assembles
bundles for each platform layout, and -- when a real usdRigView binary is
passed -- runs its --usdRig-check against the bundle for the host platform.

Run directly:  python3 packaging/test_assemble_bundle.py [usdRigView-path]
Or via ctest:  testRigExecBundlePackaging.
"""

from __future__ import annotations

import importlib.util
import json
import os
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ASSEMBLE = HERE / "assemble_bundle.py"
# Optional first positional argument: a built usdRigView binary (ctest
# passes $<TARGET_FILE:usdRigView>). Consumed here, not by unittest.
LAUNCHER = None
if len(sys.argv) > 1 and not sys.argv[1].startswith("-"):
    LAUNCHER = Path(sys.argv[1])
    del sys.argv[1]
HOST_PLATFORM = {"win32": "windows", "darwin": "macos"}.get(
    sys.platform, "linux")
IMAGING = {"windows": "rigExecImaging.dll",
           "macos": "librigExecImaging.dylib",
           "linux": "librigExecImaging.so"}[HOST_PLATFORM]


def touch(path: Path, text: str = "x\n") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def make_trees(base: Path, platform: str) -> dict:
    usd = base / "usd-install"
    touch(usd / "bin" / "usdview", "# usdview\n")
    touch(usd / "lib" / "usd_usd.dll", "lib\n")
    if platform == "windows":
        touch(usd / "Lib" / "site-packages" / "pxr" / "__init__.py")
    else:
        touch(usd / "lib" / "python3.11" / "site-packages" / "pxr" /
              "__init__.py")
    rig = base / "rig-install"
    imaging = {"windows": "rigExecImaging.dll",
               "macos": "librigExecImaging.dylib",
               "linux": "librigExecImaging.so"}[platform]
    touch(rig / "lib" / imaging, "lib\n")
    for plug in ("rigExecSchema", "rigExecImaging"):
        touch(rig / "lib" / "usd" / plug / "resources" / "plugInfo.json",
              "{}\n")
    touch(rig / "lib" / "python" / "rigexec" / "__init__.py")
    touch(rig / "lib" / "python" / "rigExecUsdview" / "editorExtension.py")
    python_home = base / "python-home"
    if platform == "windows":
        touch(python_home / "python.exe", "py\n")
        touch(python_home / "Lib" / "os.py")
        for name in ("msvcp140.dll", "vcruntime140.dll"):
            touch(python_home / "Lib" / "site-packages" / "PySide6" / name,
                  "runtime\n")
    else:
        touch(python_home / "bin" / "python3", "py\n")
        touch(python_home / "lib" / "python3.11" / "os.py")
    source = base / "checkout"
    for plugin in ("touchPose", "shapeEditor"):
        touch(source / "plugin" / plugin / "plugInfo.json", "{}\n")
        touch(source / "plugin" / plugin / "module.py")
    for name in ("rigexec_flat.usda", "ArmShotAnim.usda", "ArmRig.usda"):
        touch(source / "examples" / name, "#usda 1.0\n")
    for name in ("LICENSE", "NOTICE", "THIRD_PARTY_NOTICES.md"):
        touch(source / name, f"{name}\n")
    build = base / "build"
    tool = "rigExecPose.exe" if platform == "windows" else "rigExecPose"
    touch(build / tool, "tool\n")
    touch(source / "icons" / "control.png")
    return {"usd": usd, "rig": rig, "python": python_home,
            "source": source, "build": build}


FRAMEWORK_PYTHON = "/Library/Frameworks/Python.framework/Versions/3.11/Python"


def build_framework_python(python_home: Path, extra_dep: bool = False,
                           stub: bool = False, missing_name="libmissing.dylib"):
    cc = shutil.which("clang") or shutil.which("cc")
    (python_home / "bin").mkdir(parents=True, exist_ok=True)
    (python_home / "lib" / "python3.11").mkdir(parents=True,
                                               exist_ok=True)
    touch(python_home / "lib" / "python3.11" / "os.py")
    lib_src = python_home / "answer.c"
    lib_src.write_text("int usdrig_answer(void) { return 42; }\n",
                       encoding="utf-8")
    lib = python_home / "lib" / "libpython3.11.dylib"


    # Exercise both slices because python.org distributes universal binaries.
    fat = ["-arch", "arm64", "-arch", "x86_64"]
    subprocess.run([cc, "-dynamiclib", "-install_name", FRAMEWORK_PYTHON,
                    "-o", str(lib), str(lib_src)] + fat, check=True)
    main_src = python_home / "main.c"
    stub_decl = ('__attribute__((used)) static const char stub_path[] = '
                 '"Resources/Python.app/Contents/MacOS/Python";\n'
                 if stub else "")
    main_src.write_text(
        '#include <stdio.h>\n'
        "int usdrig_answer(void);\n" + stub_decl +
        'int main(void) { printf("%d\\n", usdrig_answer()); return 0; }\n',
        encoding="utf-8")
    link = [str(lib)]
    if extra_dep:
        dep_src = python_home / "dep.c"
        dep_src.write_text("int usdrig_dep(void) { return 1; }\n",
                           encoding="utf-8")

        dep = python_home.parent / missing_name
        subprocess.run([cc, "-dynamiclib", "-install_name",
                        f"/opt/homebrew/lib/{missing_name}",
                        "-o", str(dep), str(dep_src)] + fat, check=True)
        link.append(str(dep))
    subprocess.run([cc, str(main_src), "-o",
                    str(python_home / "bin" / "python3")] + link + fat,
                   check=True)


def assemble(base: Path, platform: str, launcher: Path, archive=True,
             customize=None):
    trees = make_trees(base, platform)
    if customize is not None:
        customize(trees)
    output = base / f"bundle-{platform}"
    cmd = [sys.executable, str(ASSEMBLE),
           "--usd-install", str(trees["usd"]),
           "--rig-install", str(trees["rig"]),
           "--python-home", str(trees["python"]),
           "--launcher", str(launcher),
           "--output", str(output),
           "--version", "v1.2.0",
           "--platform", platform,
           "--platform-slug", f"{platform}-x64",
           "--source-root", str(trees["source"]),
           "--rig-build-dir", str(trees["build"]),
           "--git-sha", "deadbeef"]
    if archive:
        suffix = ".zip" if platform == "windows" else ".tar.gz"
        cmd += ["--archive", str(base / f"bundle-{platform}{suffix}")]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    return proc, output, trees


class AssembleBundleTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.base = Path(self._tmp.name)
        self.launcher = LAUNCHER
        self.dummy_launcher = self.base / "dummy-launcher"
        self.dummy_launcher.write_bytes(b"launcher\n")

    def tearDown(self):
        self._tmp.cleanup()

    def _launcher(self):
        return self.launcher or self.dummy_launcher

    def test_windows_layout_and_zip(self):
        proc, output, _ = assemble(self.base, "windows", self._launcher())
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertTrue((output / "usdRigView.exe").is_file())
        self.assertTrue((output / "app" / "usd" / "bin" / "usdview").is_file())
        self.assertTrue((output / "app" / "python" / "python.exe").is_file())
        self.assertTrue((output / "app" / "rig" / "bin" /
                         "rigExecPose.exe").is_file())
        self.assertTrue((output / "app" / "plugins" / "touchPose" /
                         "plugInfo.json").is_file())
        self.assertTrue((output / "app" / "examples" /
                         "rigexec_flat.usda").is_file())
        self.assertTrue((output / "app" / "examples" / "ArmRig.usda").is_file())
        self.assertTrue((output / "app" / "icons" / "control.png").is_file())
        for lib in ("usd", "rig"):
            for name in ("msvcp140.dll", "vcruntime140.dll"):
                self.assertTrue(
                    (output / "app" / lib / "lib" / name).is_file(),
                    f"MSVC runtime not shipped in app/{lib}/lib: {name}")
        manifest = json.loads(
            (output / "bundle-manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["version"], "1.2.0")
        self.assertEqual(manifest["platform"], "windows-x64")
        self.assertEqual(manifest["git_sha"], "deadbeef")
        archive = self.base / "bundle-windows.zip"
        self.assertTrue(archive.is_file())
        with zipfile.ZipFile(archive) as zf:
            names = zf.namelist()
        self.assertTrue(any(n.endswith("usdRigView.exe") for n in names))
        self.assertTrue(any("bundle-manifest.json" in n for n in names))

    def test_macos_app_layout_and_tarball(self):
        proc, output, _ = assemble(self.base, "macos", self._launcher())
        self.assertEqual(proc.returncode, 0, proc.stderr)
        app = output / "usdRigView.app" / "Contents"
        self.assertTrue((app / "MacOS" / "usdRigView").is_file())
        plist = (app / "Info.plist").read_text(encoding="utf-8")
        self.assertIn("org.usdrig.usdRigView", plist)
        self.assertIn("<string>1.2.0</string>", plist)
        self.assertTrue((app / "Resources" / "app" / "usd" / "bin" /
                         "usdview").is_file())
        with tarfile.open(self.base / "bundle-macos.tar.gz") as tf:
            names = tf.getnames()
        self.assertTrue(any("usdRigView.app/Contents/MacOS/usdRigView" in n
                            for n in names))

    def test_linux_layout_and_tarball(self):
        proc, output, _ = assemble(self.base, "linux", self._launcher())
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertTrue((output / "usdRigView").is_file())
        self.assertTrue((output / "app" / "python" / "bin" /
                         "python3").is_file())
        with tarfile.open(self.base / "bundle-linux.tar.gz") as tf:
            self.assertTrue(any(n.endswith("/usdRigView")
                                for n in tf.getnames()))

    @unittest.skipUnless(os.name == "posix", "symlinks need POSIX")
    def test_copy_tree_preserves_symlinks_when_asked(self):
        spec = importlib.util.spec_from_file_location("assemble_bundle",
                                                      ASSEMBLE)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        src = self.base / "fw-src" / "Python.framework" / "Versions"
        touch(src / "3.11" / "Python", "binary\n")
        (src / "Current").symlink_to("3.11", target_is_directory=True)
        kept = self.base / "kept"
        module.copy_tree(src.parent.parent, kept, trim_dev=False,
                         preserve_symlinks=True)
        self.assertTrue(
            (kept / "Python.framework" / "Versions" / "Current").is_symlink())
        copied = self.base / "copied"
        module.copy_tree(src.parent.parent, copied, trim_dev=False)
        self.assertFalse(
            (copied / "Python.framework" / "Versions" / "Current").is_symlink())
        # A link escaping the tree (setup-python's bin/python3 ->
        # /Library/Frameworks/...) becomes a real copy, not a link
        # back to the build machine.
        outside = self.base / "outside-interpreter"
        touch(outside, "interpreter\n")
        (src.parent.parent / "bin").mkdir(parents=True, exist_ok=True)
        (src.parent.parent / "bin" / "python3").symlink_to(outside)
        bundled = self.base / "bundled"
        module.copy_tree(src.parent.parent, bundled, trim_dev=False,
                         preserve_symlinks=True)
        copied_python = bundled / "bin" / "python3"
        self.assertFalse(copied_python.is_symlink())
        self.assertEqual(copied_python.read_text(encoding="utf-8"),
                         "interpreter\n")

    @unittest.skipUnless(os.name == "posix", "symlinks need POSIX")
    def test_trim_dev_removes_framework_headers_alias(self):
        spec = importlib.util.spec_from_file_location("assemble_bundle", ASSEMBLE)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        src = self.base / "python-framework"
        touch(src / "include" / "python3.11" / "Python.h")
        (src / "Headers").symlink_to("include/python3.11")
        for trim in (False, True):
            dst = self.base / ("trimmed" if trim else "complete")
            module.copy_tree(src, dst, trim_dev=trim, preserve_symlinks=True)
            self.assertEqual((dst / "include").exists(), not trim)
            self.assertEqual((dst / "Headers").is_symlink(), not trim)
            self.assertEqual((dst / "Headers").exists(), not trim)

    @unittest.skipUnless(
        sys.platform == "darwin" and (shutil.which("clang") or
                                      shutil.which("cc")) and
        shutil.which("otool") and shutil.which("install_name_tool"),
        "macOS Mach-O relocation needs clang, otool, install_name_tool")
    def test_macos_bundle_relocates_framework_python(self):
        def customize(trees):
            build_framework_python(trees["python"])

        proc, output, _ = assemble(self.base, "macos", self._launcher(),
                                   archive=False, customize=customize)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        python = (output / "usdRigView.app" / "Contents" / "Resources" /
                  "app" / "python" / "bin" / "python3")
        deps = subprocess.run(["otool", "-L", str(python)],
                              capture_output=True, text=True).stdout
        self.assertNotIn("/Library/Frameworks/Python.framework", deps)
        self.assertIn("@loader_path/../lib/libpython3.11.dylib", deps)
        run = subprocess.run([str(python)], capture_output=True, text=True)
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertEqual(run.stdout.strip(), "42")

    @unittest.skipUnless(
        sys.platform == "darwin" and (shutil.which("clang") or
                                      shutil.which("cc")) and
        shutil.which("otool") and shutil.which("install_name_tool"),
        "macOS Mach-O relocation needs clang, otool, install_name_tool")
    def test_macos_framework_shim_home_bundles_python_app(self):
        def customize(trees):
            framework = (self.base / "Python.framework" / "Versions" /
                         "3.11")
            build_framework_python(framework, stub=True)
            library = framework / "lib" / "libpython3.11.dylib"
            library.rename(framework / "Python")
            library.symlink_to("../Python")
            app_exe = (framework / "Resources" / "Python.app" /
                       "Contents" / "MacOS" / "Python")
            app_exe.parent.mkdir(parents=True)
            shutil.copy2(framework / "bin" / "python3", app_exe)
            shim = trees["python"]
            (shim / "bin" / "python3").unlink()
            (shim / "bin" / "python3").symlink_to(
                framework / "bin" / "python3")

        proc, output, _ = assemble(self.base, "macos", self._launcher(),
                                   archive=False, customize=customize)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        app_python = (output / "usdRigView.app" / "Contents" /
                      "Resources" / "app" / "python")
        self.assertTrue((app_python / "Python").is_file())
        self.assertTrue((app_python / "Resources" / "Python.app" /
                         "Contents" / "MacOS" / "Python").is_file())
        deps = subprocess.run(["otool", "-L",
                               str(app_python / "bin" / "python3")],
                              capture_output=True, text=True).stdout
        self.assertIn("@loader_path/../Python", deps)
        self.assertNotIn("Python.app", deps)
        run = subprocess.run([str(app_python / "bin" / "python3")],
                             capture_output=True, text=True)
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertEqual(run.stdout.strip(), "42")

    @unittest.skipUnless(
        sys.platform == "darwin" and (shutil.which("clang") or
                                      shutil.which("cc")) and
        shutil.which("otool") and shutil.which("install_name_tool"),
        "macOS Mach-O relocation needs clang, otool, install_name_tool")
    def test_macos_bundle_rejects_missing_python_app_stub_target(self):
        def customize(trees):
            build_framework_python(trees["python"], stub=True)

        proc, _, _ = assemble(self.base, "macos", self._launcher(),
                              archive=False, customize=customize)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("Python.app", proc.stderr)

    @unittest.skipUnless(
        sys.platform == "darwin" and (shutil.which("clang") or
                                      shutil.which("cc")) and
        shutil.which("otool") and shutil.which("install_name_tool"),
        "macOS Mach-O relocation needs clang, otool, install_name_tool")
    def test_macos_bundle_rejects_unbundled_absolute_dep(self):
        def customize(trees):
            build_framework_python(trees["python"], extra_dep=True)

        proc, _, _ = assemble(self.base, "macos", self._launcher(),
                              archive=False, customize=customize)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("libmissing.dylib", proc.stderr)

    @unittest.skipUnless(
        sys.platform == "darwin" and shutil.which("clang") and
        shutil.which("otool") and shutil.which("install_name_tool"),
        "macOS Mach-O relocation tools required")
    def test_optional_sql_dependency_is_required_outside_qt_plugin(self):
        def customize(trees):
            build_framework_python(trees["python"], extra_dep=True,
                                   missing_name="libpq.5.dylib")

        proc, _, _ = assemble(self.base, "macos", self._launcher(),
                             archive=False, customize=customize)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("libpq.5.dylib", proc.stderr)

    def test_missing_inputs_report_every_problem(self):
        empty = self.base / "empty"
        empty.mkdir()
        cmd = [sys.executable, str(ASSEMBLE),
               "--usd-install", str(empty), "--rig-install", str(empty),
               "--python-home", str(empty), "--launcher", str(empty / "no"),
               "--output", str(self.base / "out"), "--version", "1.2.0",
               "--platform", "linux", "--source-root", str(empty)]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(proc.returncode, 2)
        for needle in ("bin/usdview", "imaging library", "plugInfo",
                       "Python home", "launcher not found", "touchPose"):
            self.assertIn(needle, proc.stderr)

    def test_real_launcher_check_mode(self):
        if not self.launcher:
            self.skipTest("no usdRigView binary passed")
        proc, output, _ = assemble(self.base, HOST_PLATFORM, self.launcher,
                                   archive=False)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        if HOST_PLATFORM == "macos":
            exe = (output / "usdRigView.app" / "Contents" / "MacOS" /
                   "usdRigView")
        elif HOST_PLATFORM == "windows":
            exe = output / "usdRigView.exe"
        else:
            exe = output / "usdRigView"
        ok = subprocess.run([str(exe), "--usdRig-check"],
                            capture_output=True, text=True)
        self.assertEqual(ok.returncode, 0, ok.stderr)
        self.assertIn("bundle OK", ok.stdout)

        if HOST_PLATFORM == "macos":
            imaging = (output / "usdRigView.app" / "Contents" /
                       "Resources" / "app" / "rig" / "lib" / IMAGING)
        else:
            imaging = output / "app" / "rig" / "lib" / IMAGING
        env_result = subprocess.run([str(exe), "--usdRig-print-env"],
                                    capture_output=True, text=True)
        self.assertEqual(env_result.returncode, 0, env_result.stderr)
        env_values = dict(line.split("=", 1)
                          for line in env_result.stdout.splitlines())
        self.assertEqual(env_values["PYTHONDONTWRITEBYTECODE"], "1")
        editor_dir = imaging.parent / "python" / "rigExecUsdview"
        self.assertIn(editor_dir.resolve(),
                      [Path(entry).resolve() for entry in
                       env_values["PYTHONPATH"].split(os.pathsep)])
        # usdview only loads the editor's plug-in container when its
        # plugInfo.json directory is registered.
        self.assertIn(editor_dir.resolve(),
                      [Path(entry).resolve() for entry in
                       env_values["PXR_PLUGINPATH_NAME"].split(os.pathsep)])
        if HOST_PLATFORM == "windows":
            # noodles.dll installs into rig/bin; the pinned DLL search
            # has to reach it or UsdNoodles fails to load.
            tools_dir = imaging.parent.parent / "bin"
            self.assertIn(tools_dir.resolve(),
                          [Path(entry).resolve() for entry in
                           env_values["PXR_USD_WINDOWS_DLL_PATH"]
                           .split(os.pathsep)])
        imaging.unlink()
        broken = subprocess.run([str(exe), "--usdRig-check"],
                                capture_output=True, text=True)
        self.assertEqual(broken.returncode, 2)
        self.assertIn("imaging library", broken.stderr)

    @unittest.skipUnless(
        sys.platform == "darwin" and (shutil.which("clang") or
                                      shutil.which("cc")),
        "launcher Mach-O check needs a macOS host with clang")
    def test_real_launcher_check_rejects_absolute_python_dep(self):
        if not self.launcher:
            self.skipTest("no usdRigView binary passed")
        proc, output, _ = assemble(self.base, HOST_PLATFORM, self.launcher,
                                   archive=False)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        app = (output / "usdRigView.app" / "Contents" / "Resources" /
               "app")
        exe = (output / "usdRigView.app" / "Contents" / "MacOS" /
               "usdRigView")
        build_framework_python(app / "python")
        broken = subprocess.run([str(exe), "--usdRig-check"],
                                capture_output=True, text=True)
        self.assertEqual(broken.returncode, 2, broken.stdout)
        self.assertIn("Python.framework", broken.stderr)

    def test_real_launcher_env_isolation(self):
        if not self.launcher:
            self.skipTest("no usdRigView binary passed")
        proc, output, _ = assemble(self.base, HOST_PLATFORM, self.launcher,
                                   archive=False)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        if HOST_PLATFORM == "macos":
            exe = (output / "usdRigView.app" / "Contents" / "MacOS" /
                   "usdRigView")
            app = (output / "usdRigView.app" / "Contents" / "Resources" /
                   "app")
        elif HOST_PLATFORM == "windows":
            exe = output / "usdRigView.exe"
            app = output / "app"
        else:
            exe = output / "usdRigView"
            app = output / "app"
        bogus = str(self.base / "bogus-ambient")
        env = dict(os.environ)
        env["PYTHONPATH"] = bogus
        env["PXR_PLUGINPATH_NAME"] = bogus
        env["PYTHONHOME"] = bogus
        printed = subprocess.run([str(exe), "--usdRig-print-env"], env=env,
                                 capture_output=True, text=True)
        self.assertEqual(printed.returncode, 0, printed.stderr)
        values = dict(line.split("=", 1) for line in
                      printed.stdout.splitlines() if "=" in line)
        for name in ("PYTHONPATH", "PXR_PLUGINPATH_NAME", "PYTHONHOME"):
            self.assertIn(name, values)
            self.assertNotIn(bogus, values[name],
                             f"ambient {name} leaked into the bundle env")
        self.assertEqual(Path(values["PYTHONHOME"]), app / "python")
        if HOST_PLATFORM == "windows":
            self.assertIn("PXR_USD_WINDOWS_DLL_PATH", values)
            dll_path = values["PXR_USD_WINDOWS_DLL_PATH"]
            self.assertNotIn(bogus, dll_path)
            self.assertIn(str(app / "usd" / "lib"), dll_path)
            self.assertIn(str(app / "rig" / "lib"), dll_path)


if __name__ == "__main__":
    unittest.main(verbosity=2)
