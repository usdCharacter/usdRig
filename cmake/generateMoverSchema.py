"""Generate USD-free attribute metadata/defaults and optional codeless USD resources.

Called by rigexec_add_mover. The input schema.usda remains the source of
attribute types and defaults.

The generator reads through USD Python when it imports and otherwise reads
the schema directly from its USDA text, so a Python-less OpenUSD install
(CI's first gate) still builds. The text reader supports the documented
mover shape only: one concrete class inheriting </Typed>, attribute and
relationship statements, and a GLOBAL over carrying libraryName and
skipCodeGeneration. Anything outside that shape fails with an error naming
the construct, so the fallback never silently drops schema content.

The fallback emits declarations without documentation metadata (docs need
usdGenSchema); attribute types, defaults, variability, allowedTokens, and
extraPlugInfo are preserved.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import runpy
import struct
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


def _default_lines(attributes, emitted=None):
    """attributes holds (name, kind, uniform, has_value, value) records
    sorted by name from either reader."""
    lines = []
    names = set()
    for name, kind, uniform, has_value, value in attributes:
        cpp_name = identifier(name)
        if cpp_name in names:
            raise ValueError(f"Attribute identifier collision: {name}")
        names.add(cpp_name)
        if emitted is not None:
            emitted.append("{" + json.dumps(name) + ", " + json.dumps(kind) +
                           ", " + ("true" if uniform else "false") + "}")
        if not has_value:
            continue
        element = kind[:-2] if kind.endswith("[]") else kind
        if element not in SCALARS:
            raise ValueError(f"Unsupported mover default type: {name}: {kind}")
        cpp_type = SCALARS[element][0]
        if kind.endswith("[]"):
            cpp_type = f"std::array<{cpp_type}, {len(value)}>"
        lines.append(f"inline constexpr {cpp_type} {cpp_name} = {literal(value, kind)};")
    return lines


def emit_header(type_name, namespace, attributes, reads=()):
    """Shared header emission. reads holds (type, attributes) pairs for other
    schema types the adapter reads; their defaults go in nested namespaces."""
    lines = ["// Generated from schema.usda; do not edit.", "#pragma once",
             '#include "rigExecBinary/external.h"', "#include <array>",
             "#include <cstdint>", "#include <string_view>",
             f"namespace rigExec {{ namespace {namespace} {{",
             f"inline constexpr char Type[] = {json.dumps(type_name)};",
             "struct Attribute { const char *name; const char *type; bool uniform; };"]
    emitted = []
    lines += _default_lines(attributes, emitted)
    for read_type, read_attributes in reads:
        lines.append(f"namespace {identifier(read_type)} {{")
        lines += _default_lines(read_attributes)
        lines.append("}")
    lines += [f"inline constexpr std::array<Attribute, {len(emitted)}> Attributes = {{{{",
              ",\n".join("    " + a for a in emitted), "}};", "}} // namespace rigExec", ""]
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# USD-free schema text reader.
# ---------------------------------------------------------------------------

_SIMPLE_ESCAPES = {
    "n": "\n", "t": "\t", "b": "\b", "f": "\f", "r": "\r", "v": "\v",
    "0": "\0", '"': '"', "'": "'", "\\": "\\",
}

_NUMBER_RE = re.compile(r"[+-]?(?:0[xX][0-9a-fA-F_]+"
                        r"|(?:\d[\d_]*\.[\d_]*|\.\d[\d_]*|\d[\d_]*)"
                        r"(?:[eE][+-]?\d+)?)")
_IDENT_START = frozenset(
    "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ_")
_IDENT_CHARS = frozenset(
    "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ_0123456789")


def _f32(value):
    try:
        return struct.unpack("<f", struct.pack("<f", value))[0]
    except OverflowError:
        return math.copysign(math.inf, value)


class _UsdaText:
    """A USDA subset reader: specs, metadata, and property values."""

    def __init__(self, text, path):
        # A caller-supplied display path; errors name it with a line number.
        self.text = text
        self.path = path
        self.pos = 0
        self.end = len(text)

    def fail(self, message):
        line = self.text.count("\n", 0, self.pos) + 1
        raise ValueError(f"{self.path}:{line}: {message}")

    def skip_space(self):
        text, end = self.text, self.end
        pos = self.pos
        while pos < end:
            char = text[pos]
            if char in " \t\r\n":
                pos += 1
            elif char == "#":
                while pos < end and text[pos] != "\n":
                    pos += 1
            else:
                break
        self.pos = pos

    def peek(self):
        self.skip_space()
        if self.pos < self.end:
            return self.text[self.pos]
        return ""

    def expect(self, char):
        self.skip_space()
        if self.pos >= self.end or self.text[self.pos] != char:
            self.fail(f"expected {char!r}")
        self.pos += 1

    def at_end(self):
        return self.peek() == ""

    def _read_word(self, namespaced):
        self.skip_space()
        text, end = self.text, self.end
        pos = self.pos
        if pos >= end or text[pos] not in _IDENT_START:
            self.fail("expected an identifier")
        start = pos
        while pos < end and text[pos] in _IDENT_CHARS:
            pos += 1
        if namespaced:
            # Property names carry namespaces: inputs:gain.
            while pos + 1 < end and text[pos] == ":" and \
                    text[pos + 1] in _IDENT_START:
                pos += 1
                while pos < end and text[pos] in _IDENT_CHARS:
                    pos += 1
        self.pos = pos
        return text[start:pos]

    def read_identifier(self):
        return self._read_word(namespaced=True)

    def read_bare_identifier(self):
        return self._read_word(namespaced=False)

    def read_string(self):
        self.skip_space()
        text, end = self.text, self.end
        pos = self.pos
        if pos >= end or text[pos] not in "\"'":
            self.fail("expected a string")
        quote = text[pos]
        triple = text[pos:pos + 3] == quote * 3
        pos += 3 if triple else 1
        closer = quote * 3 if triple else quote
        chars = []
        while True:
            if pos >= end:
                self.fail("unterminated string")
            if triple and text.startswith(closer, pos):
                pos += 3
                break
            if not triple and text[pos] == quote:
                pos += 1
                break
            char = text[pos]
            if char == "\\":
                pos += 1
                if pos >= end:
                    self.fail("unterminated string")
                escaped = text[pos]
                if escaped in _SIMPLE_ESCAPES:
                    chars.append(_SIMPLE_ESCAPES[escaped])
                    pos += 1
                elif escaped in "xX":
                    digits = text[pos + 1:pos + 3]
                    if len(digits) != 2 or any(
                            c not in "0123456789abcdefABCDEF" for c in digits):
                        self.fail("bad \\x escape")
                    chars.append(chr(int(digits, 16)))
                    pos += 3
                elif escaped in "uU":
                    width = 4 if escaped == "u" else 8
                    digits = text[pos + 1:pos + 1 + width]
                    if len(digits) != width or any(
                            c not in "0123456789abcdefABCDEF" for c in digits):
                        self.fail("bad unicode escape")
                    chars.append(chr(int(digits, 16)))
                    pos += 1 + width
                else:
                    chars.append(escaped)
                    pos += 1
            else:
                chars.append(char)
                pos += 1
        self.pos = pos
        return "".join(chars)

    def read_asset(self):
        self.expect("@")
        text, end = self.text, self.end
        pos = self.pos
        chars = []
        while True:
            if pos >= end:
                self.fail("unterminated asset path")
            if text[pos] == "@":
                if text[pos:pos + 2] == "@@":
                    chars.append("@")
                    pos += 2
                    continue
                pos += 1
                break
            chars.append(text[pos])
            pos += 1
        self.pos = pos
        return "".join(chars)

    def read_path(self):
        self.expect("<")
        text, end = self.text, self.end
        pos = self.pos
        start = pos
        while pos < end and text[pos] != ">":
            pos += 1
        if pos >= end:
            self.fail("unterminated path")
        self.pos = pos + 1
        return "<" + text[start:pos] + ">"

    def read_number(self):
        self.skip_space()
        match = _NUMBER_RE.match(self.text, self.pos)
        if not match:
            self.fail("expected a number")
        token = match.group(0).replace("_", "")
        self.pos = match.end()
        if re.fullmatch(r"[+-]?0[xX][0-9a-fA-F]+", token):
            return int(token, 16)
        if re.fullmatch(r"[+-]?\d+", token):
            return int(token)
        return float(token)

    def read_value(self):
        char = self.peek()
        if char == "(":
            return self.read_tuple()
        if char == "[":
            return self.read_array()
        if char == "{":
            return self.read_dictionary()
        if char == "<":
            return ("path", self.read_path())
        if char == "@":
            return ("asset", self.read_asset())
        if char in "\"'":
            return self.read_string()
        if char.isdigit() or char in "+-.":
            # A sign may introduce inf/nan rather than digits.
            rest = self.text[self.pos:]
            bare = re.match(r"[+-]?(inf(?:inity)?|nan)\b", rest, re.IGNORECASE)
            if bare:
                self.pos += bare.end()
                word = bare.group(1).lower()
                number = math.inf if word.startswith("inf") else math.nan
                return -number if rest.startswith("-") else number
            return self.read_number()
        if char in _IDENT_START:
            word = self.read_bare_identifier()
            if word == "true":
                return True
            if word == "false":
                return False
            if word == "None":
                return None
            if word.lower() in ("inf", "infinity"):
                return math.inf
            if word.lower() == "nan":
                return math.nan
            self.fail(f"unexpected identifier {word!r} in value")
        self.fail("expected a value")

    def read_tuple(self):
        self.expect("(")
        values = []
        if self.peek() == ")":
            self.pos += 1
            return values
        while True:
            values.append(self.read_value())
            char = self.peek()
            if char == ",":
                self.pos += 1
                if self.peek() == ")":
                    self.pos += 1
                    return values
            elif char == ")":
                self.pos += 1
                return values
            else:
                self.fail("expected ',' or ')'")

    def read_array(self):
        self.expect("[")
        values = []
        if self.peek() == "]":
            self.pos += 1
            return values
        while True:
            values.append(self.read_value())
            char = self.peek()
            if char == ",":
                self.pos += 1
                if self.peek() == "]":
                    self.pos += 1
                    return values
            elif char == "]":
                self.pos += 1
                return values
            else:
                self.fail("expected ',' or ']'")

    def read_dictionary(self):
        self.expect("{")
        entries = {}
        while self.peek() != "}":
            if self.peek() == ";":
                self.pos += 1
                continue
            first = self.read_identifier()
            self.skip_space()
            if self.peek() == "=":
                key = first
            else:
                # A typed entry: string key = value. The value parser is
                # generic, so the declared type needs no interpretation.
                self.read_array_suffix_opt()
                key = self.read_identifier()
                self.skip_space()
            self.expect("=")
            entries[key] = self.read_value()
            self.skip_comma_opt()
        self.pos += 1
        return entries

    def read_array_suffix_opt(self):
        self.skip_space()
        if self.pos + 1 < self.end and self.text[self.pos] == "[" and \
                self.text[self.pos + 1] == "]":
            self.pos += 2
            return True
        return False

    def skip_comma_opt(self):
        self.skip_space()
        while self.peek() == ",":
            self.pos += 1
            self.skip_space()

    def read_metadata(self):
        """A (...) block: key = value pairs, bare doc strings, and commas."""
        self.expect("(")
        entries = {}
        while self.peek() != ")":
            char = self.peek()
            if char == "," or char == ";":
                self.pos += 1
                continue
            if char in "\"'":
                self.read_string()
                continue
            key = self.read_identifier()
            if key in ("prepend", "append", "add"):
                # One file holds one opinion, so a list op over a single
                # statement reads the same as the assignment itself.
                key = self.read_identifier()
            elif key == "delete":
                self.fail("the text reader does not support 'delete' opinions")
            self.skip_space()
            self.expect("=")
            entries[key] = self.read_value()
            self.skip_comma_opt()
        self.pos += 1
        return entries

    def skip_balanced(self, opener, closer):
        self.skip_space()
        depth = 0
        while self.pos < self.end:
            char = self.text[self.pos]
            if char == "#":
                self.skip_space()
                continue
            if char in "\"'":
                self.read_string()
                continue
            if char == "@":
                self.read_asset()
                continue
            if char == opener:
                depth += 1
            elif char == closer:
                depth -= 1
                if depth == 0:
                    self.pos += 1
                    return
            self.pos += 1
        self.fail(f"unterminated {opener!r}")


def _read_text_schema(schema_path, type_name):
    """Parse the mover class members, its header, and GLOBAL customData.

    Returns (class_ident, members, inherits, class_data, global_data).
    members preserves file order: ("attr", (name, kind, uniform, custom,
    has_value, value, allowed)) or ("rel", (name, custom, targets))."""
    text = Path(schema_path).read_text(encoding="utf-8")
    reader = _UsdaText(text, str(schema_path))
    reader.skip_space()
    if reader.at_end():
        reader.fail("empty schema file")
    if reader.peek() == "(":
        # Layer metadata (sublayers are checked separately).
        reader.skip_balanced("(", ")")
    members = []
    class_ident = None
    inherits = None
    class_data = {}
    global_data = {}
    saw_type = False
    while not reader.at_end():
        spec = reader.read_bare_identifier()
        if spec not in ("class", "def", "over"):
            reader.fail(f"unexpected {spec!r}; the text reader supports "
                        "class, def, and over specs only")
        ident = None
        if reader.peek() in "\"'":
            name = reader.read_string()
        else:
            ident = reader.read_bare_identifier()
            if reader.peek() in "\"'":
                name = reader.read_string()
            else:
                reader.fail("expected a quoted prim name")
        metadata = {}
        if reader.peek() == "(":
            metadata = reader.read_metadata()
        if name == "GLOBAL":
            if isinstance(metadata.get("customData"), dict):
                global_data.update(metadata["customData"])
            if reader.peek() == "{":
                reader.expect("{")
                while reader.peek() != "}":
                    if reader.peek() == ";":
                        reader.pos += 1
                        continue
                    key = reader.read_identifier()
                    reader.skip_space()
                    reader.expect("=")
                    value = reader.read_value()
                    if key == "customData" and isinstance(value, dict):
                        global_data.update(value)
                reader.expect("}")
            elif reader.peek() == ";":
                reader.pos += 1
            continue
        if spec != "class" or name != type_name:
            # Another class (the shared schema holds many): skip it.
            if reader.peek() == "{":
                reader.skip_balanced("{", "}")
            elif reader.peek() == ";":
                reader.pos += 1
            continue
        if saw_type:
            reader.fail(f"duplicate class {type_name}")
        saw_type = True
        class_ident = ident
        inherits = metadata.get("inherits")
        if isinstance(metadata.get("customData"), dict):
            class_data.update(metadata["customData"])
        if reader.peek() == ";":
            reader.pos += 1
            continue
        reader.expect("{")
        while reader.peek() != "}":
            if reader.peek() == ";":
                reader.pos += 1
                continue
            word = reader.read_bare_identifier()
            if word == "rel":
                custom = False
                saved = reader.pos
                probe = reader.read_identifier()
                following = reader.peek()
                if probe == "custom" and following and \
                        following in _IDENT_START:
                    custom = True
                    rel_name = reader.read_identifier()
                else:
                    reader.pos = saved
                    rel_name = reader.read_identifier()
                targets = None
                if reader.peek() == "=":
                    reader.pos += 1
                    targets = reader.read_value()
                if reader.peek() == "(":
                    reader.read_metadata()
                members.append(("rel", (rel_name, custom, targets)))
                continue
            custom = uniform = False
            while word in ("custom", "uniform"):
                if word == "custom":
                    custom = True
                else:
                    uniform = True
                word = reader.read_bare_identifier()
            kind = word
            if reader.read_array_suffix_opt():
                kind += "[]"
            prop = reader.read_identifier()
            has_value = False
            value = None
            if reader.peek() == "=":
                reader.pos += 1
                value = reader.read_value()
                has_value = True
            allowed = None
            if reader.peek() == "(":
                attr_meta = reader.read_metadata()
                if "variability" in attr_meta:
                    reader.fail("the text reader supports the 'uniform' "
                                "keyword, not 'variability' metadata")
                if attr_meta.get("allowedTokens") is not None:
                    allowed = attr_meta["allowedTokens"]
                    if not isinstance(allowed, list) or any(
                            not isinstance(item, str) for item in allowed):
                        reader.fail("allowedTokens must be a string list")
            members.append(("attr", (prop, kind, uniform, custom, has_value,
                                     value, allowed)))
        reader.expect("}")
    if not saw_type:
        raise ValueError(f"Schema has no class {type_name}")
    return class_ident, members, inherits, class_data, global_data


def _check_text_sublayers(schema_path):
    """Refuse local composition the text reader cannot see. The built-in
    usd*/schema.usda layers define base schema classes only."""
    text = Path(schema_path).read_text(encoding="utf-8")
    reader = _UsdaText(text, str(schema_path))
    reader.skip_space()
    if reader.peek() != "(":
        return
    for key, value in reader.read_metadata().items():
        if key != "subLayers":
            continue
        layers = value if isinstance(value, list) else [value]
        for layer in layers:
            if layer is None:
                continue
            if not (isinstance(layer, tuple) and layer[0] == "asset" and
                    re.fullmatch(r"usd[A-Za-z]*/schema\.usda", layer[1])):
                reader.fail("the text reader does not compose sublayers; "
                            "a Python-enabled OpenUSD is required")


_FLOAT_VECTOR_ARITY = {
    "float2": 2, "float3": 3, "point3f": 3, "vector3f": 3,
    "normal3f": 3, "color3f": 3, "double3": 3, "int3": 3,
}


def _convert_scalar(kind, value, name):
    if kind == "bool":
        if not isinstance(value, bool):
            raise ValueError(f"Unsupported mover default type: {name}: {kind}")
        return value
    if kind == "int":
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"Unsupported mover default type: {name}: {kind}")
        return value
    if kind in ("float", "double"):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"Unsupported mover default type: {name}: {kind}")
        number = float(value)
        return _f32(number) if kind == "float" else number
    if kind == "token":
        if not isinstance(value, str):
            raise ValueError(f"Unsupported mover default type: {name}: {kind}")
        return value
    if kind in _FLOAT_VECTOR_ARITY:
        if not isinstance(value, list) or len(value) != _FLOAT_VECTOR_ARITY[kind]:
            raise ValueError(f"Unsupported mover default type: {name}: {kind}")
        scalar = "double" if kind == "double3" else "int" if kind == "int3" else "float"
        return [_convert_scalar(scalar, item, name) for item in value]
    if kind == "matrix4d":
        if not isinstance(value, list) or len(value) != 4 or any(
                not isinstance(row, list) or len(row) != 4 for row in value):
            raise ValueError(f"Unsupported mover default type: {name}: {kind}")
        return [[_convert_scalar("double", item, name) for item in row]
                for row in value]
    raise ValueError(f"Unsupported mover default type: {name}: {kind}")


def _convert_default(kind, value, name):
    if kind.endswith("[]"):
        if not isinstance(value, list):
            raise ValueError(f"Unsupported mover default type: {name}: {kind}")
        return [_convert_scalar(kind[:-2], item, name) for item in value]
    return _convert_scalar(kind, value, name)


def _shortest_float(value, parse):
    for precision in range(1, 18):
        text = "%.*g" % (precision, value)
        if parse(text) == value:
            return text
    return repr(value)


def _usda_float(value):
    return _shortest_float(value, lambda text: _f32(float(text)))


def _usda_double(value):
    return _shortest_float(value, float)


def _usda_scalar(kind, value):
    if kind == "bool":
        return "1" if value else "0"
    if kind == "int":
        return str(int(value))
    if kind == "float":
        return _usda_float(float(value))
    if kind == "double":
        return _usda_double(float(value))
    if kind == "token":
        return json.dumps(str(value), ensure_ascii=False)
    if kind == "matrix4d":
        rows = ["(" + ", ".join(_usda_double(float(item)) for item in row) + ")"
                for row in value]
        return "( " + ", ".join(rows) + " )"
    if kind == "double3":
        render = _usda_double
    elif kind == "int3":
        render = str
    else:
        render = _usda_float
    return "(" + ", ".join(render(item) for item in value) + ")"


def _usda_value(kind, value):
    if kind.endswith("[]"):
        return "[" + ", ".join(_usda_scalar(kind[:-2], item) for item in value) + "]"
    return _usda_scalar(kind, value)


def _usda_targets(targets):
    if targets is None:
        return None
    if isinstance(targets, tuple) and targets[0] == "path":
        return targets[1]
    if isinstance(targets, list):
        return "[" + ", ".join(_usda_targets(item) for item in targets) + "]"
    raise ValueError(f"Unsupported relationship target: {targets!r}")


def emit_generated_schema(type_name, class_ident, members):
    """Codeless schema resources without usdGenSchema. Declarations only:
    documentation metadata needs a Python-enabled OpenUSD."""
    lines = ["#usda 1.0", "(",
             '    "WARNING: THIS FILE IS GENERATED BY usdGenSchema.  DO NOT EDIT."',
             ")", ""]
    if class_ident is None:
        lines.append(f'class "{type_name}"')
    else:
        lines.append(f'class {class_ident} "{type_name}"')
    lines.append("{")
    # usdGenSchema sorts every property by name, relationships included.
    ordered = sorted(members, key=lambda item: item[1][0])
    for member, record in ordered:
        if member == "rel":
            name, custom, targets = record
            line = "    "
            if custom:
                line += "custom "
            rendered = _usda_targets(targets)
            lines.append(line + f"rel {name}" +
                         ("" if rendered is None else f" = {rendered}"))
            continue
        name, kind, uniform, custom, has_value, value, allowed = record
        line = "    "
        if custom:
            line += "custom "
        if uniform:
            line += "uniform "
        line += f"{kind} {name}"
        if has_value:
            line += f" = {_usda_value(kind, value)}"
        if allowed is not None:
            line += " ("
            lines.append(line)
            lines.append("        allowedTokens = [" +
                         ", ".join(json.dumps(item, ensure_ascii=False)
                                   for item in allowed) + "]")
            lines.append("    )")
        else:
            lines.append(line)
    lines += ["}", "", ""]
    return "\n".join(lines)


def emit_plug_info(type_name, global_data, inherits, class_data):
    library = global_data.get("libraryName")
    if not library or not isinstance(library, str):
        raise ValueError("Mover schemas must set GLOBAL customData libraryName")
    paths = inherits if isinstance(inherits, list) else [inherits]
    if len(paths) != 1 or not (
            isinstance(paths[0], tuple) and paths[0][0] == "path" and
            paths[0][1] == "</Typed>"):
        raise ValueError(
            f"Mover schema {type_name} inherits {paths!r}; the text reader "
            "supports only </Typed> (a Python-enabled OpenUSD is required)")
    extra = class_data.get("extraPlugInfo", {})
    if not isinstance(extra, dict):
        raise ValueError(f"Mover schema {type_name}: extraPlugInfo must be a dictionary")
    # The registered type name is the library prefix plus the class name,
    # each defaulting as usdGenSchema defaults them; a name that differs
    # from the schema identifier carries a UsdSchemaBase alias for it.
    prefix = global_data.get("libraryPrefix") or library
    class_name = class_data.get("className") or type_name
    if not isinstance(prefix, str) or not isinstance(class_name, str):
        raise ValueError(f"Mover schema {type_name}: libraryPrefix and "
                         "className must be strings")
    tf_type = prefix + class_name
    entry = {"autoGenerated": True, "bases": ["UsdTyped"],
             "schemaIdentifier": type_name, "schemaKind": "concreteTyped"}
    if tf_type != type_name:
        entry["alias"] = {"UsdSchemaBase": type_name}
    for key, value in extra.items():
        try:
            json.dumps(value)
        except (TypeError, ValueError):
            raise ValueError(f"Mover schema {type_name}: extraPlugInfo "
                             f"{key!r} is not JSON data")
        entry[key] = value
    entry = dict(sorted(entry.items()))
    info = {"Plugins": [{"Info": {"Types": {tf_type: entry}},
                         "Name": library, "ResourcePath": ".",
                         "Root": ".", "Type": "resource"}]}
    return json.dumps(info, indent=2) + "\n"


def _import_pxr(usd):
    usd = Path(usd).resolve()
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
    return Ar, Plug, Sdf, Usd, dll_dirs


def _generate_with_pxr(args, namespace):
    Ar, Plug, Sdf, Usd, dll_dirs = _import_pxr(args.usd)
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

    def records(type_name):
        prim = stage.GetPrimAtPath("/" + type_name)
        if not prim or not prim.IsAbstract():
            raise ValueError(f"Schema has no class {type_name}")
        attributes = []
        for attr in sorted(prim.GetAttributes(), key=lambda a: a.GetName()):
            name, kind = attr.GetName(), str(attr.GetTypeName())
            uniform = attr.GetVariability() == Sdf.VariabilityUniform
            value = attr.Get()
            attributes.append((name, kind, uniform, value is not None, value))
        return attributes

    reads = [(read_type, records(read_type)) for read_type in args.read_type]
    write_if_changed(args.header,
                     emit_header(args.type, namespace, records(args.type), reads))
    if args.resources:
        global_prim = stage.GetPrimAtPath("/GLOBAL")
        if not global_prim or global_prim.GetCustomDataByKey("skipCodeGeneration") is not True:
            raise ValueError("Mover schemas must set GLOBAL customData skipCodeGeneration = true")
        resources = Path(args.resources).resolve()
        resources.mkdir(parents=True, exist_ok=True)
        generator = str(Path(args.usd).resolve() / "bin/usdGenSchema")
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


def _convert_record(record):
    name, kind, uniform, custom, has_value, value, allowed = record
    if has_value:
        value = _convert_default(kind, value, name)
    return (name, kind, uniform, custom, has_value, value, allowed)


def _text_records(members):
    records = []
    for member, record in members:
        if member != "attr":
            continue
        name, kind, uniform, custom, has_value, value, allowed = record
        records.append((name, kind, uniform, has_value, value))
    records.sort(key=lambda item: item[0])
    return records


def _generate_from_text(args, namespace):
    _check_text_sublayers(args.schema)
    class_ident, members, inherits, class_data, global_data = \
        _read_text_schema(args.schema, args.type)
    converted = [(member, _convert_record(record) if member == "attr" else record)
                 for member, record in members]
    records = _text_records(converted)
    reads = []
    for read_type in args.read_type:
        read_members = _read_text_schema(args.schema, read_type)[1]
        reads.append((read_type, _text_records(
            (member, _convert_record(record))
            for member, record in read_members if member == "attr")))
    write_if_changed(args.header,
                     emit_header(args.type, namespace, records, reads))
    if args.resources:
        if global_data.get("skipCodeGeneration") is not True:
            raise ValueError("Mover schemas must set GLOBAL customData skipCodeGeneration = true")
        # usdGenSchema writes every class it finds; the text resources
        # below describe one class only.
        if _count_text_classes(args.schema) != 1:
            raise ValueError("The text reader writes resources for one class "
                             "per schema file (a Python-enabled OpenUSD is required)")
        for member, record in converted:
            if member == "rel" and record[2] is not None:
                raise ValueError(f"Relationship targets on "
                                 f"</{args.type}.{record[0]}> cannot be "
                                 "specified in a schema.")
        resources = Path(args.resources).resolve()
        resources.mkdir(parents=True, exist_ok=True)
        write_if_changed(resources / "generatedSchema.usda",
                         emit_generated_schema(args.type, class_ident, converted))
        write_if_changed(resources / "plugInfo.json",
                         emit_plug_info(args.type, global_data, inherits, class_data))


def _count_text_classes(schema_path):
    text = Path(schema_path).read_text(encoding="utf-8")
    reader = _UsdaText(text, str(schema_path))
    reader.skip_space()
    if reader.peek() == "(":
        reader.skip_balanced("(", ")")
    count = 0
    while not reader.at_end():
        spec = reader.read_bare_identifier()
        if spec == "class":
            count += 1
        if reader.peek() in "\"'":
            reader.read_string()
        else:
            reader.read_bare_identifier()
            if reader.peek() in "\"'":
                reader.read_string()
        if reader.peek() == "(":
            reader.skip_balanced("(", ")")
        if reader.peek() == "{":
            reader.skip_balanced("{", "}")
        elif reader.peek() == ";":
            reader.pos += 1
    return count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", required=True)
    parser.add_argument("--schema", required=True)
    parser.add_argument("--type", required=True)
    parser.add_argument("--namespace", required=True)
    parser.add_argument("--header", required=True)
    parser.add_argument("--resources")
    parser.add_argument("--read-type", action="append", default=[])
    args = parser.parse_args()
    namespace = identifier(args.namespace)
    use_pxr = os.environ.get("RIGEXEC_MOVER_SCHEMA_TEXT_ONLY") != "1"
    if use_pxr and args.resources and not (
            Path(args.usd) / "bin" / "usdGenSchema").exists():
        print("generateMoverSchema: no usdGenSchema under --usd; "
              "reading the schema as text", file=sys.stderr)
        use_pxr = False
    if use_pxr:
        try:
            _generate_with_pxr(args, namespace)
            return
        except ImportError as error:
            print(f"generateMoverSchema: pxr unavailable ({error}); "
                  "reading the schema as text", file=sys.stderr)
    _generate_from_text(args, namespace)


if __name__ == "__main__":
    main()
