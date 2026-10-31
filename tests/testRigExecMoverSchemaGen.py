#!/usr/bin/env python3
"""Fallback and parity coverage for cmake/generateMoverSchema.py.

Part 1 always runs and needs no USD install: the USD-free text reader
regenerates a rich mover schema byte-identically to the goldens below,
and rejects out-of-shape schemas with explicit errors.

Part 2 runs when the USD install under argv[1] imports pxr: the text
reader must match the pxr/usdGenSchema path (identical header and
plugInfo bytes; identical declarations in the generated schema). It
skips honestly where USD has no Python bindings; part 1 and the C++
external-mover suite still cover the fallback there.

Usage: testRigExecMoverSchemaGen.py <usd-install-dir>
"""
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

RIG = Path(__file__).resolve().parents[1]
SCRIPT = RIG / "cmake" / "generateMoverSchema.py"
TYPE = "FallbackTestMover"
NAMESPACE = "FallbackTestSchema"

FIXTURE = """#usda 1.0
(
    subLayers = [@usd/schema.usda@]
)
over "GLOBAL" (
    customData = {
        string libraryName = "fallbackTestSchema"
        string libraryPath = "fallbackTestSchema"
        string libraryPrefix = "Fallback"
        bool skipCodeGeneration = true
    }
) {}

class FallbackTestMover "FallbackTestMover" (
    inherits = </Typed>
    customData = {
        string className = "FallbackTestMover"
        dictionary extraPlugInfo = {
            string zebra = "z-last"
            string apple = "a-first"
        }
    }
    doc = \"\"\"The first sentence. The second has a "quoted" word, (parens),
    {braces}, and an escaped backslash: \\\\."\"\"
) {
    rel rigExec:aFirstRel (
        doc = 'Single-quoted doc with (parens) and [brackets].'
    )
    float inputs:gain = 1 (
        doc = \"\"\"Gain applied to every point. Range [0, 10].\"\"\"
    )
    uniform bool rigExec:flag = true
    token inputs:mode = "fast" (
        allowedTokens = ["fast", "slow"]
    )
    float3 inputs:offset = (0, 1.5, -2)
    float2 inputs:uv = (0.25, 0.5)
    double inputs:rate = 0.5
    int inputs:count = 3
    uniform int rigExec:windowFrames = 16
    float[] inputs:weights = [0.5, 1]
    token[] inputs:names = ["a", "b"]
    float3[] inputs:points = [(0, 0, 0), (1, 2, 3)]
    point3f inputs:point = (1, 2, 3)
    vector3f inputs:dir = (0, 0, 1)
    normal3f inputs:nrm = (0, 1, 0)
    color3f inputs:tint = (1, 0.5, 0.25)
    double3 inputs:scale3 = (1, 2, 3)
    int3 inputs:tile = (4, 5, 6)
    matrix4d inputs:xf = ((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1))
    float inputs:nodefault
    rel rigExec:reference
    string inputs:label
}
"""

EXPECTED_HEADER = '''// Generated from schema.usda; do not edit.
#pragma once
#include "rigExecBinary/external.h"
#include <array>
#include <cstdint>
#include <string_view>
namespace rigExec { namespace FallbackTestSchema {
inline constexpr char Type[] = "FallbackTestMover";
struct Attribute { const char *name; const char *type; bool uniform; };
inline constexpr int32_t inputs_count = 3;
inline constexpr std::array<float, 3> inputs_dir = {{0.0f, 0.0f, 1.0f}};
inline constexpr float inputs_gain = 1.0f;
inline constexpr std::string_view inputs_mode = "fast";
inline constexpr std::array<std::string_view, 2> inputs_names = {{"a", "b"}};
inline constexpr std::array<float, 3> inputs_nrm = {{0.0f, 1.0f, 0.0f}};
inline constexpr std::array<float, 3> inputs_offset = {{0.0f, 1.5f, -2.0f}};
inline constexpr std::array<float, 3> inputs_point = {{1.0f, 2.0f, 3.0f}};
inline constexpr std::array<std::array<float, 3>, 2> inputs_points = {{{{0.0f, 0.0f, 0.0f}}, {{1.0f, 2.0f, 3.0f}}}};
inline constexpr double inputs_rate = 0.5;
inline constexpr std::array<double, 3> inputs_scale3 = {{1.0, 2.0, 3.0}};
inline constexpr std::array<int32_t, 3> inputs_tile = {{4, 5, 6}};
inline constexpr std::array<float, 3> inputs_tint = {{1.0f, 0.5f, 0.25f}};
inline constexpr std::array<float, 2> inputs_uv = {{0.25f, 0.5f}};
inline constexpr std::array<float, 2> inputs_weights = {{0.5f, 1.0f}};
inline constexpr std::array<double, 16> inputs_xf = {{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
inline constexpr bool rigExec_flag = true;
inline constexpr int32_t rigExec_windowFrames = 16;
inline constexpr std::array<Attribute, 20> Attributes = {{
    {"inputs:count", "int", false},
    {"inputs:dir", "vector3f", false},
    {"inputs:gain", "float", false},
    {"inputs:label", "string", false},
    {"inputs:mode", "token", false},
    {"inputs:names", "token[]", false},
    {"inputs:nodefault", "float", false},
    {"inputs:nrm", "normal3f", false},
    {"inputs:offset", "float3", false},
    {"inputs:point", "point3f", false},
    {"inputs:points", "float3[]", false},
    {"inputs:rate", "double", false},
    {"inputs:scale3", "double3", false},
    {"inputs:tile", "int3", false},
    {"inputs:tint", "color3f", false},
    {"inputs:uv", "float2", false},
    {"inputs:weights", "float[]", false},
    {"inputs:xf", "matrix4d", false},
    {"rigExec:flag", "bool", true},
    {"rigExec:windowFrames", "int", true}
}};
}} // namespace rigExec
'''

EXPECTED_SCHEMA = '''#usda 1.0
(
    "WARNING: THIS FILE IS GENERATED BY usdGenSchema.  DO NOT EDIT."
)

class FallbackTestMover "FallbackTestMover"
{
    int inputs:count = 3
    vector3f inputs:dir = (0, 0, 1)
    float inputs:gain = 1
    string inputs:label
    token inputs:mode = "fast" (
        allowedTokens = ["fast", "slow"]
    )
    token[] inputs:names = ["a", "b"]
    float inputs:nodefault
    normal3f inputs:nrm = (0, 1, 0)
    float3 inputs:offset = (0, 1.5, -2)
    point3f inputs:point = (1, 2, 3)
    float3[] inputs:points = [(0, 0, 0), (1, 2, 3)]
    double inputs:rate = 0.5
    double3 inputs:scale3 = (1, 2, 3)
    int3 inputs:tile = (4, 5, 6)
    color3f inputs:tint = (1, 0.5, 0.25)
    float2 inputs:uv = (0.25, 0.5)
    float[] inputs:weights = [0.5, 1]
    matrix4d inputs:xf = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
    rel rigExec:aFirstRel
    uniform bool rigExec:flag = 1
    rel rigExec:reference
    uniform int rigExec:windowFrames = 16
}

'''

EXPECTED_PLUGINFO = '''{
  "Plugins": [
    {
      "Info": {
        "Types": {
          "FallbackFallbackTestMover": {
            "alias": {
              "UsdSchemaBase": "FallbackTestMover"
            },
            "apple": "a-first",
            "autoGenerated": true,
            "bases": [
              "UsdTyped"
            ],
            "schemaIdentifier": "FallbackTestMover",
            "schemaKind": "concreteTyped",
            "zebra": "z-last"
          }
        }
      },
      "Name": "fallbackTestSchema",
      "ResourcePath": ".",
      "Root": ".",
      "Type": "resource"
    }
  ]
}
'''


def run_generator(schema_text, schema_type=TYPE, with_resources=True,
                  text_only=True, usd_dir=None):
    """Run the generator on schema_text; return (exit, stdout, files dict)."""
    work = Path(tempfile.mkdtemp(prefix="mover-schema-gen"))
    schema = work / "schema.usda"
    schema.write_text(schema_text, encoding="utf-8", newline="\n")
    header = work / "test.h"
    command = [sys.executable, "-S", str(SCRIPT), "--usd",
               usd_dir or str(work / "empty-usd"),
               "--schema", str(schema), "--type", schema_type,
               "--namespace", NAMESPACE, "--header", str(header)]
    resources = work / "resources"
    if with_resources:
        command += ["--resources", str(resources)]
    env = dict(os.environ)
    if text_only:
        env["RIGEXEC_MOVER_SCHEMA_TEXT_ONLY"] = "1"
    else:
        env.pop("RIGEXEC_MOVER_SCHEMA_TEXT_ONLY", None)
    done = subprocess.run(command, capture_output=True, text=True, env=env)
    files = {}
    if header.exists():
        files["header"] = header.read_text(encoding="utf-8")
    generated = resources / "generatedSchema.usda"
    if generated.exists():
        files["schema"] = generated.read_text(encoding="utf-8")
    plug_info = resources / "plugInfo.json"
    if plug_info.exists():
        files["plugInfo"] = plug_info.read_text(encoding="utf-8")
    return done.returncode, done.stderr, files


def check_fallback_goldens():
    status, stderr, files = run_generator(FIXTURE)
    assert status == 0, f"fallback failed:\n{stderr}"
    assert files.get("header") == EXPECTED_HEADER, "header golden differs"
    assert files.get("schema") == EXPECTED_SCHEMA, "schema golden differs"
    assert files.get("plugInfo") == EXPECTED_PLUGINFO, "plugInfo golden differs"
    # The goldens are valid JSON and USD where applicable.
    json.loads(files["plugInfo"])
    print("goldens match")


def check_fallback_without_flag():
    """The CI path: no TEXT_ONLY flag, a Python-less --usd, and the
    generator still succeeds through the fallback, saying so."""
    status, stderr, files = run_generator(FIXTURE, text_only=False)
    assert status == 0, f"unflagged fallback failed:\n{stderr}"
    assert files.get("header") == EXPECTED_HEADER, "header golden differs"
    assert files.get("schema") == EXPECTED_SCHEMA, "schema golden differs"
    assert files.get("plugInfo") == EXPECTED_PLUGINFO, "plugInfo golden differs"
    assert "reading the schema as text" in stderr, \
        f"fallback note missing:\n{stderr}"
    print("unflagged fallback matches goldens")


def check_fallback_errors():
    def fails(name, schema_text, fragment, with_resources=True):
        status, stderr, _ = run_generator(
            schema_text, with_resources=with_resources)
        assert status != 0, f"{name}: unexpectedly succeeded"
        assert fragment in stderr, f"{name}: {fragment!r} not in:\n{stderr}"
        print(f"error ok: {name}")

    def header_ok(name, schema_text):
        status, stderr, files = run_generator(
            schema_text, with_resources=False)
        assert status == 0, f"{name}: header unexpectedly failed:\n{stderr}"
        assert "header" in files, f"{name}: no header written"
        print(f"header ok: {name}")

    status, stderr, _ = run_generator(FIXTURE, schema_type="Nope")
    assert status != 0 and "Schema has no class Nope" in stderr, \
        f"missing class:\n{stderr}"
    print("error ok: missing class")

    fails("local sublayer",
          FIXTURE.replace("subLayers = [@usd/schema.usda@]",
                          "subLayers = [@./local.usda@]"),
          "does not compose sublayers")

    non_typed = FIXTURE.replace("inherits = </Typed>", "inherits = </Other>")
    header_ok("non-Typed inherits", non_typed)
    fails("non-Typed inherits resources", non_typed, "supports only </Typed>")

    fails("skipCodeGeneration",
          FIXTURE.replace("bool skipCodeGeneration = true",
                          "bool skipCodeGeneration = false"),
          "skipCodeGeneration = true")

    targeted = FIXTURE.replace("    rel rigExec:reference\n",
                               "    rel rigExec:reference = </X>\n")
    header_ok("relationship targets", targeted)
    fails("relationship targets resources", targeted,
          "cannot be specified in a schema")

    fails("unsupported default",
          FIXTURE.replace("    string inputs:label\n",
                          '    string inputs:label = "y"\n'),
          "Unsupported mover default type: inputs:label: string")

    multi = FIXTURE + '\nclass Extra "Extra" (\n    inherits = </Typed>\n) {\n}\n'
    header_ok("second class", multi)
    fails("second class resources", multi, "one class per schema file")

    fails("variability metadata",
          FIXTURE.replace('Gain applied to every point. Range [0, 10]."""',
                          'Gain applied to every point."""\n'
                          '        variability = "uniform"'),
          "supports the 'uniform' keyword")


def describe_generated(generated):
    from pxr import Usd
    stage = Usd.Stage.Open(str(generated))
    assert stage, f"cannot open {generated}"
    prim = stage.GetPrimAtPath("/" + TYPE)
    assert prim, "missing class prim"
    found = {}
    for attr in prim.GetAttributes():
        found[attr.GetName()] = (str(attr.GetTypeName()),
                                 str(attr.GetVariability()),
                                 repr(attr.Get()))
    rels = sorted(rel.GetName() for rel in prim.GetRelationships())
    return found, rels


def check_parity(usd_dir):
    """The text reader matches pxr/usdGenSchema; skips without pxr."""
    candidates = [Path(usd_dir) / "Lib/site-packages",
                  Path(usd_dir) / "lib/python"]
    sys.path[:0] = [str(p) for p in candidates if p.is_dir()]
    if hasattr(os, "add_dll_directory"):
        for directory in (Path(usd_dir) / "bin", Path(usd_dir) / "lib"):
            if directory.is_dir():
                os.add_dll_directory(str(directory))
    try:
        import pxr  # noqa: F401
    except ImportError:
        print(f"SKIP parity: no pxr under {usd_dir}")
        return
    status, stderr, pxr_files = run_generator(
        FIXTURE, text_only=False, usd_dir=usd_dir)
    assert status == 0, f"pxr path failed:\n{stderr}"
    status, stderr, text_files = run_generator(
        FIXTURE, text_only=True, usd_dir=usd_dir)
    assert status == 0, f"text path failed:\n{stderr}"
    assert pxr_files["header"] == text_files["header"], "header differs"
    assert pxr_files["plugInfo"] == text_files["plugInfo"], "plugInfo differs"
    print("header and plugInfo identical across paths")
    with tempfile.TemporaryDirectory(prefix="mover-schema-parity") as work:
        work = Path(work)
        pxr_schema = work / "pxr.usda"
        text_schema = work / "text.usda"
        pxr_schema.write_text(pxr_files["schema"], encoding="utf-8")
        text_schema.write_text(text_files["schema"], encoding="utf-8")
        pxr_attrs, pxr_rels = describe_generated(pxr_schema)
        text_attrs, text_rels = describe_generated(text_schema)
    assert pxr_rels == text_rels, f"rels differ: {pxr_rels} vs {text_rels}"
    assert set(pxr_attrs) == set(text_attrs), "attr sets differ"
    for name in pxr_attrs:
        assert pxr_attrs[name] == text_attrs[name], \
            f"{name}: {pxr_attrs[name]} vs {text_attrs[name]}"
    print(f"{len(pxr_attrs)} attributes + {len(pxr_rels)} rels identical")


def print_goldens():
    status, stderr, files = run_generator(FIXTURE)
    assert status == 0, f"fallback failed:\n{stderr}"
    for key in ("header", "schema", "plugInfo"):
        print(f"===== {key} ====")
        print(files[key], end="" if files[key].endswith("\n") else "\n")


def main():
    if "--print-goldens" in sys.argv:
        print_goldens()
        return
    usd_dir = sys.argv[1] if len(sys.argv) > 1 else ""
    check_fallback_goldens()
    check_fallback_without_flag()
    check_fallback_errors()
    check_parity(usd_dir)
    print("testRigExecMoverSchemaGen passed")


if __name__ == "__main__":
    main()
