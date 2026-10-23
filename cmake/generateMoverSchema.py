"""Generate USD-free attribute metadata/defaults and optional codeless USD resources.

Called by rigexec_add_mover. Requires the Python interpreter matching --usd.
The input schema.usda remains the source of attribute types and defaults.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import runpy
import sys
import sysconfig


def identifier(name):
    result = re.sub(r"[^a-zA-Z0-9_]", "_", name)
    if not result or result[0].isdigit():
        raise ValueError(f"Invalid C++ identifier: {name}")
    return result


SCALARS = {
    "bool": ("bool", "Bool"), "int": ("int32_t", "Int"),
    "float": ("float", "Float"), "double": ("double", "Double"),
    "token": ("std::string_view", "Token"),
    "float2": ("std::array<float, 2>", "Vec2f"),
    "float3": ("std::array<float, 3>", "Vec3f"),
    "point3f": ("std::array<float, 3>", "Vec3f"),
    "vector3f": ("std::array<float, 3>", "Vec3f"),
    "normal3f": ("std::array<float, 3>", "Vec3f"),
    "color3f": ("std::array<float, 3>", "Vec3f"),
    "double3": ("std::array<double, 3>", "Vec3d"),
    "int3": ("std::array<int32_t, 3>", "Vec3i"),
    "matrix4d": ("std::array<double, 16>", "Matrix4d"),
}


def literal(value, kind):
    if kind.endswith("[]"):
        return "{{" + ", ".join(literal(v, kind[:-2]) for v in value) + "}}"
    if kind == "bool":
        return "true" if value else "false"
    if kind == "int":
        return str(int(value))
    if kind in ("float", "double"):
        number = float(value)
        if not math.isfinite(number):
            raise ValueError("Mover schema defaults must be finite")
        return repr(number) + ("f" if kind == "float" else "")
    if kind == "token":
        return json.dumps(str(value), ensure_ascii=True)
    if kind == "matrix4d":
        values = [value[r][c] for r in range(4) for c in range(4)]
        return "{{" + ", ".join(literal(v, "double") for v in values) + "}}"
    scalar = "double" if kind == "double3" else "int" if kind == "int3" else "float"
    return "{{" + ", ".join(literal(v, scalar) for v in value) + "}}"


def write_if_changed(path, text):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists() or path.read_text(encoding="utf-8") != text:
        path.write_text(text, encoding="utf-8", newline="\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", required=True)
    parser.add_argument("--schema", required=True)
    parser.add_argument("--type", required=True)
    parser.add_argument("--namespace", required=True)
    parser.add_argument("--header", required=True)
    parser.add_argument("--resources")
    args = parser.parse_args()
    usd = Path(args.usd).resolve()
    modules = [usd / "Lib/site-packages", usd / "lib/python"]
    modules = [str(p) for p in modules if p.is_dir()]
    sys.path[:0] = modules
    # CMake invokes -S: import Jinja2 without running arbitrary .pth hooks
    # that can replace pxr with another implementation.
    for key in ("purelib", "platlib"):
        path = sysconfig.get_path(key)
        if path not in sys.path:
            sys.path.append(path)
    os.environ["PYTHONPATH"] = os.pathsep.join(modules)
    os.environ["PXR_PLUGINPATH_NAME"] = str(usd / "lib/usd")
    os.environ["USD_DISABLE_PRIM_DEFINITIONS_FOR_USDGENSCHEMA"] = "1"
    os.environ["PATH"] = os.pathsep.join([str(usd / "bin"), str(usd / "lib"), os.environ["PATH"]])
    dll_dirs = []
    if hasattr(os, "add_dll_directory"):
        dll_dirs = [os.add_dll_directory(str(p)) for p in (usd / "bin", usd / "lib") if p.is_dir()]
    from pxr import Ar, Plug, Sdf, Usd

    # Match usdGenSchema's source-schema resolution, including usd/schema.usda.
    Ar.SetPreferredResolver("ArDefaultResolver")
    registry = Plug.Registry()
    resources = {plugin.resourcePath
                 for schema_type in registry.GetAllDerivedTypes("UsdSchemaBase")
                 if (plugin := registry.GetPluginForType(schema_type))}
    Ar.DefaultResolver.SetDefaultSearchPath(sorted(resources))

    stage = Usd.Stage.Open(str(Path(args.schema).resolve()))
    if not stage:
        raise ValueError(f"Cannot open schema: {args.schema}")
    prim = stage.GetPrimAtPath("/" + args.type)
    if not prim or not prim.IsAbstract():
        raise ValueError(f"Schema has no class {args.type}")
    namespace = identifier(args.namespace)
    lines = ["// Generated from schema.usda; do not edit.", "#pragma once",
             '#include "rigExecBinary/external.h"', "#include <array>",
             "#include <cstdint>", "#include <string_view>",
             f"namespace rigExec {{ namespace {namespace} {{",
             f"inline constexpr char Type[] = {json.dumps(args.type)};",
             "struct Attribute { const char *name; const char *type; bool uniform; };" ]
    attributes = []
    names = set()
    for attr in sorted(prim.GetAttributes(), key=lambda a: a.GetName()):
        name, kind = attr.GetName(), str(attr.GetTypeName())
        cpp_name = identifier(name)
        if cpp_name in names:
            raise ValueError(f"Attribute identifier collision: {name}")
        names.add(cpp_name)
        uniform = attr.GetVariability() == Sdf.VariabilityUniform
        attributes.append("{" + json.dumps(name) + ", " + json.dumps(kind) +
                          ", " + ("true" if uniform else "false") + "}")
        value = attr.Get()
        if value is None:
            continue
        element = kind[:-2] if kind.endswith("[]") else kind
        if element not in SCALARS:
            raise ValueError(f"Unsupported mover default type: {name}: {kind}")
        cpp_type = SCALARS[element][0]
        if kind.endswith("[]"):
            cpp_type = f"std::array<{cpp_type}, {len(value)}>"
        lines.append(f"inline constexpr {cpp_type} {cpp_name} = {literal(value, kind)};")
    lines += [f"inline constexpr std::array<Attribute, {len(attributes)}> Attributes = {{{{",
              ",\n".join("    " + a for a in attributes), "}};", "}} // namespace rigExec", ""]
    write_if_changed(args.header, "\n".join(lines))
    if args.resources:
        global_prim = stage.GetPrimAtPath("/GLOBAL")
        if not global_prim or global_prim.GetCustomDataByKey("skipCodeGeneration") is not True:
            raise ValueError("Mover schemas must set GLOBAL customData skipCodeGeneration = true")
        resources = Path(args.resources).resolve()
        resources.mkdir(parents=True, exist_ok=True)
        generator = str(usd / "bin/usdGenSchema")
        sys.argv = [generator, str(Path(args.schema).resolve()), str(resources)]
        try:
            runpy.run_path(generator, run_name="__main__")
        except SystemExit as result:
            if result.code not in (None, 0):
                raise
        info_path = resources / "plugInfo.json"
        # usdGenSchema's JSON includes leading comments.
        text = info_path.read_text(encoding="utf-8")
        text = "\n".join(line for line in text.splitlines() if not line.lstrip().startswith("#"))
        info = json.loads(text)
        for plugin in info["Plugins"]:
            plugin.pop("LibraryPath", None)
            plugin["Root"] = "."
            plugin["ResourcePath"] = "."
        write_if_changed(info_path, json.dumps(info, indent=2) + "\n")
    # Keep DLL search handles alive until all USD work finishes.
    del dll_dirs


if __name__ == "__main__":
    main()
