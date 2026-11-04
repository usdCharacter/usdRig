#!/usr/bin/env python
"""
bin/_env.sh's RIGEXEC_BUILD_DIR override.

Each case sources _env.sh in a fresh bash subprocess and reads the exported
environment back:
  * unset: the build-dependent variables name build/, and
    RIGEXEC_IMAGING_DLL and RIGEXEC_BUILD_DIR stay unset;
  * set (relative, with a trailing slash, absolute): PATH, the loader path,
    PYTHONPATH, the three schema/imaging/mover plugin paths and
    RIGEXEC_IMAGING_DLL name the selected tree and no longer build/;
    RIGEXEC_BUILD_DIR is exported absolute; a caller's RIGEXEC_IMAGING_DLL
    and BUILD are kept;
  * strict mode: `set -euo pipefail; . bin/_env.sh` succeeds with the
    override unset and set, with and without RIGEXEC_IMAGING_DLL and the
    loader variables in the environment;
  * rigexec_build builds a configured tree, default or selected, with
    cmake --build <tree> (a fake cmake records its arguments), and exits 1
    with an error when that build fails.

Needs bash; macOS or Linux. Usage: test_env_build_dir.py
"""
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))
ENV_SH = os.path.join(REPO, "bin", "_env.sh")

_failures = []
_groups = 0

# Variables a caller may bring that the cases control explicitly.
_CONTROLLED = ("RIGEXEC_BUILD_DIR", "RIGEXEC_IMAGING_DLL", "DYLD_LIBRARY_PATH",
               "LD_LIBRARY_PATH", "PXR_PLUGINPATH_NAME", "PYTHONPATH", "BUILD",
               "RIG", "USD", "PY", "VENV")
_FIXED_PATH = "/usr/bin:/bin:/usr/sbin:/sbin"


def _Check(condition, message):
    if not condition:
        _failures.append(message)


def _BaseEnv(**extra):
    env = {k: v for k, v in os.environ.items() if k not in _CONTROLLED}
    env["PATH"] = _FIXED_PATH + (":" + os.path.dirname(shutil.which("c++"))
                                 if shutil.which("c++") else "")
    env.update(extra)
    return env


def _Source(env, script="", envSh=ENV_SH, strict=False, loader=None):
    """Sources `envSh` in bash under `env`, runs `script`, then prints the
    exported environment NUL-separated. Returns (returncode, dict, stderr).

    The environment is printed with bash builtins, and `loader` (a value for
    DYLD_LIBRARY_PATH / LD_LIBRARY_PATH) is exported inside the script: on
    macOS, System Integrity Protection strips DYLD_* from the environment of
    every protected binary it executes (/bin/bash, /usr/bin/env)."""
    prefix = "set -euo pipefail; " if strict else ""
    if loader is not None:
        prefix += 'export %s="%s"; ' % (_LoaderVariable(), loader)
    dump = ('for _n in $(compgen -e); do printf \'%s=%s\\0\' "$_n" "${!_n}"; '
            'done')
    command = '%s. "%s"; %s\n%s' % (prefix, envSh, script, dump)
    proc = subprocess.run(["bash", "-c", command], env=env,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    exported = {}
    for item in proc.stdout.split(b"\0"):
        if b"=" in item:
            key, _, value = item.partition(b"=")
            exported[key.decode()] = value.decode(errors="replace")
    return proc.returncode, exported, proc.stderr.decode(errors="replace")


def _Entries(value):
    return [entry for entry in (value or "").split(":") if entry]


def _BuildVariables(env):
    return {name: env.get(name, "") for name in
            ("PATH", "DYLD_LIBRARY_PATH", "LD_LIBRARY_PATH", "PYTHONPATH",
             "PXR_PLUGINPATH_NAME")}


def _LoaderVariable():
    return "DYLD_LIBRARY_PATH" if sys.platform == "darwin" else \
        "LD_LIBRARY_PATH"


def _LibraryName():
    return "librigExecImaging.dylib" if sys.platform == "darwin" else \
        "librigExecImaging.so"


def _CheckNamesTree(env, tree, label, checkout=REPO):
    """Every build-dependent entry names `tree` and none names the
    checkout's build/."""
    default = os.path.join(checkout, "build")
    wanted = {
        "PATH": [tree],
        _LoaderVariable(): [tree],
        "PYTHONPATH": [os.path.join(tree, "python")],
        "PXR_PLUGINPATH_NAME": [
            os.path.join(tree, "usd", "rigExecSchema", "resources"),
            os.path.join(tree, "usd", "rigExecImaging", "resources"),
            os.path.join(tree, "usd", "rigExecMoverPlugins")],
    }
    for name, entries in wanted.items():
        have = _Entries(env.get(name))
        for entry in entries:
            _Check(entry in have, "%s: %s lacks %s (%s)"
                   % (label, name, entry, have[:6]))
        stale = [e for e in have
                 if e == default or e.startswith(default + os.sep)]
        _Check(not stale, "%s: %s still names build/: %s"
               % (label, name, stale))
    _Check(_Entries(env.get("PATH"))[0] == tree,
           "%s: PATH does not start with the tree" % label)


def _TestUnset():
    global _groups
    env = _BaseEnv(PYTHONPATH="/opt/x/py", PXR_PLUGINPATH_NAME="/opt/x/plug")
    rc, out, err = _Source(env, loader="/opt/x/lib")
    _Check(rc == 0, "unset: sourcing failed: %s" % err)
    build = os.path.join(REPO, "build")
    _Check(_Entries(out.get("PATH"))[0] == build,
           "unset: PATH does not start with build/")
    loader = _LoaderVariable()
    usd = os.path.join(os.path.dirname(REPO), "usd-install")
    _Check(out.get(loader) == "%s/lib:%s:/opt/x/lib" % (usd, build),
           "unset: %s = %r" % (loader, out.get(loader)))
    _Check(_Entries(out.get("PYTHONPATH"))[:2] ==
           [os.path.join(REPO, "plugin", "rigExecUsdview"),
            os.path.join(build, "python")],
           "unset: PYTHONPATH = %r" % out.get("PYTHONPATH"))
    plugins = _Entries(out.get("PXR_PLUGINPATH_NAME"))
    schema = os.path.join(build, "usd", "rigExecSchema", "resources")
    start = plugins.index(schema) if schema in plugins else -1
    _Check(start >= 0 and plugins[start:start + 4] ==
           [schema, os.path.join(build, "usd", "rigExecImaging", "resources"),
            os.path.join(build, "usd", "rigExecMoverPlugins"),
            os.path.join(REPO, "plugin", "rigExecUsdview")] and
           plugins[-1] == "/opt/x/plug",
           "unset: PXR_PLUGINPATH_NAME = %r" % out.get("PXR_PLUGINPATH_NAME"))
    _Check("RIGEXEC_IMAGING_DLL" not in out,
           "unset: RIGEXEC_IMAGING_DLL exported")
    _Check("RIGEXEC_BUILD_DIR" not in out, "unset: RIGEXEC_BUILD_DIR exported")
    _Check(not any(k.startswith("_RIGEXEC") for k in out),
           "unset: an internal _RIGEXEC variable is exported")
    # An empty value is the same as unset.
    rc, empty, err = _Source(dict(env, RIGEXEC_BUILD_DIR=""),
                             loader="/opt/x/lib")
    _Check(rc == 0 and _BuildVariables(empty) == _BuildVariables(out),
           "empty RIGEXEC_BUILD_DIR differs from unset")
    _groups += 1


def _TestSet():
    global _groups
    rig = tempfile.mkdtemp(prefix="rigexec-envtest-")
    try:
        tree = os.path.join(rig, "build-envtest")
        os.makedirs(tree)
        library = os.path.join(tree, _LibraryName())
        open(library, "w").close()
        for value in ("build-envtest", "build-envtest/", tree):
            label = "set %r" % value
            rc, out, err = _Source(_BaseEnv(RIG=rig, RIGEXEC_BUILD_DIR=value,
                                            BUILD="/launcher/own/build"))
            _Check(rc == 0, "%s: sourcing failed: %s" % (label, err))
            _Check(out.get("RIGEXEC_BUILD_DIR") == tree,
                   "%s: RIGEXEC_BUILD_DIR = %r" % (
                       label, out.get("RIGEXEC_BUILD_DIR")))
            _CheckNamesTree(out, tree, label, rig)
            _Check(out.get("BUILD") == "/launcher/own/build",
                   "%s: BUILD changed to %r" % (label, out.get("BUILD")))
            _Check(out.get("RIGEXEC_IMAGING_DLL") == library,
                   "%s: RIGEXEC_IMAGING_DLL = %r" % (
                       label, out.get("RIGEXEC_IMAGING_DLL")))
        # A caller's RIGEXEC_IMAGING_DLL wins.
        rc, out, err = _Source(_BaseEnv(
            RIG=rig, RIGEXEC_BUILD_DIR="build-envtest",
            RIGEXEC_IMAGING_DLL="/elsewhere/lib.dylib"))
        _Check(rc == 0 and
               out.get("RIGEXEC_IMAGING_DLL") == "/elsewhere/lib.dylib",
               "set: a preset RIGEXEC_IMAGING_DLL was replaced: %r"
               % out.get("RIGEXEC_IMAGING_DLL"))
        # A tree without a library exports none.
        empty = os.path.join(rig, "build-empty")
        os.makedirs(empty)
        rc, out, err = _Source(_BaseEnv(RIG=rig, RIGEXEC_BUILD_DIR=empty))
        _Check(rc == 0, "set (empty tree): sourcing failed: %s" % err)
        _Check("RIGEXEC_IMAGING_DLL" not in out,
               "set (empty tree): RIGEXEC_IMAGING_DLL exported")
        _CheckNamesTree(out, empty, "set (empty tree)", rig)
    finally:
        shutil.rmtree(rig, ignore_errors=True)
    _groups += 1


def _TestStrict():
    global _groups
    loader = _LoaderVariable()
    for override in (None, "build-envtest"):
        for dll in (None, "/elsewhere/lib.dylib"):
            for loaderValue in (None, "/opt/x/lib"):
                extra = {}
                if override is not None:
                    extra["RIGEXEC_BUILD_DIR"] = override
                if dll is not None:
                    extra["RIGEXEC_IMAGING_DLL"] = dll
                rc, out, err = _Source(_BaseEnv(**extra), strict=True,
                                       loader=loaderValue)
                _Check(rc == 0 and "PXR_PLUGINPATH_NAME" in out and
                       out.get(loader, "").endswith(loaderValue or ""),
                       "strict mode, %s, %s=%s: rc %d: %s"
                       % (extra, loader, loaderValue, rc, err[-400:]))
    _groups += 1


_FAKE_RECORDER = """#!/bin/bash
printf '%s' "$(basename "$0")" >> "$RECORD"
for a in "$@"; do printf ' [%s]' "$a" >> "$RECORD"; done
printf '\\n' >> "$RECORD"
"""
_FAKE_FAILING = _FAKE_RECORDER + "exit 3\n"


def _WriteTool(directory, name, text):
    path = os.path.join(directory, name)
    with open(path, "w") as handle:
        handle.write(text)
    os.chmod(path, 0o755)


def _Configured(tree):
    os.makedirs(tree)
    open(os.path.join(tree, "CMakeCache.txt"), "w").close()


def _TestBuild():
    global _groups
    scratch = tempfile.mkdtemp(prefix="rigexec-envtest-")
    try:
        rig = os.path.join(scratch, "checkout")
        selected = os.path.join(rig, "build-envtest")
        _Configured(selected)
        _Configured(os.path.join(rig, "build"))
        record = os.path.join(scratch, "record.txt")
        fakes = {}
        for case, text in (("ok", _FAKE_RECORDER), ("fail", _FAKE_FAILING)):
            fake = os.path.join(scratch, "bin-" + case)
            os.makedirs(fake)
            _WriteTool(fake, "cmake", text)
            os.symlink(shutil.which("basename"),
                       os.path.join(fake, "basename"))
            fakes[case] = fake

        def Run(case, **extra):
            if os.path.exists(record):
                os.remove(record)
            # PATH is the fake directory alone, so the real cmake is out of
            # reach.
            script = 'export PATH="%s"; export RECORD="%s"; rigexec_build; ' \
                     'echo BUILD_RETURNED >&2' % (fakes[case], record)
            rc, out, err = _Source(_BaseEnv(RIG=rig, **extra), script=script)
            lines = open(record).read().splitlines() \
                if os.path.exists(record) else []
            return rc, err, lines

        rc, err, lines = Run("ok", RIGEXEC_BUILD_DIR="build-envtest")
        _Check(rc == 0 and "BUILD_RETURNED" in err and
               lines == ["cmake [--build] [%s]" % selected],
               "build (selected tree): rc %d, recorded %s, %s"
               % (rc, lines, err[-300:]))
        rc, err, lines = Run("ok")
        _Check(rc == 0 and "BUILD_RETURNED" in err and
               lines == ["cmake [--build] [%s/build]" % rig],
               "build (default tree): rc %d, recorded %s, %s"
               % (rc, lines, err[-300:]))
        rc, err, lines = Run("fail", RIGEXEC_BUILD_DIR="build-envtest")
        _Check(rc == 1 and "ERROR: build" in err and
               "BUILD_RETURNED" not in err,
               "build (failing cmake): rc %d, %s" % (rc, err[-300:]))
    finally:
        shutil.rmtree(scratch, ignore_errors=True)
    _groups += 1


def main():
    if not shutil.which("bash"):
        print("SKIP: no bash")
        return 0
    _TestUnset()
    _TestSet()
    _TestStrict()
    _TestBuild()
    if _failures:
        for failure in _failures:
            print("FAIL: %s" % failure)
        return 1
    print("ENV_BUILD_DIR_OK (%d groups)" % _groups)
    return 0


if __name__ == "__main__":
    sys.exit(main())
