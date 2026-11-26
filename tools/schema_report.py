#!/usr/bin/env python3
"""Reports how far the schema generated from op registration is from
libs/rigExecSchema/schema.usda, or with --diff checks the ops whose schema
is Generated against it. Nothing is written to the repository.

Inputs:
  --tool PATH    the rigExecGenSchema executable to run, or
  --ops PATH     the text it wrote (--tool wins; report mode only)
  --schema PATH  schema.usda (a tolerant parser; pxr is not imported)
  --out PATH     also write the output here
  --diff         compare the ops' authoring metadata with the schema
  --self-test    check the comparison against synthetic ops and exit
Report output: per op the schema class it declares or most plausibly
corresponds to, the properties each side lacks and the type disagreements,
then a summary table and the classes and ops without a counterpart. The exit
status is 0 whenever a report was produced.
Diff output: per op whose schema is Generated, every difference between the
class the op describes and the class in the schema (presence, type, uniform,
fallback, allowed tokens, documentation, base); per HandAuthored op, the
same for the properties it declares. Ops that declare one class must
describe it alike (see disagreements). Internal and undeclared ops are
skipped. The exit status is 1 on any difference.
"""
import argparse
import difflib
import json
import os
import re
import subprocess
import sys

USD_TYPE = (
    r"(?:bool|uchar|int|uint|int64|uint64|half|float|double|string|token|asset"
    r"|timecode|(?:int|float|double|half)[234]|matrix[234][df]|quat[fdh]"
    r"|point3[fdh]|normal3[fdh]|vector3[fdh]|color[34][fdh]|texCoord2[fdh]"
    r"|rel)(?:\[\])?"
)
PROP = re.compile(
    r"^    (?:(?:custom|uniform|prepend|append|add|delete)\s+)*"
    r"(" + USD_TYPE + r")\s+([A-Za-z_][A-Za-z0-9_:.]*)"
)
CLASS = re.compile(r'^class\s+(?:\w+\s+)?"([^"]+)"')
INHERITS = re.compile(r"inherits\s*=\s*</?([A-Za-z0-9_]+)>?")
OP_CLASS = re.compile(r'^class "([^"]+)" \(  # op (\S+) v(\d+)(?: (\w+))?')
OP_PROP = re.compile(
    r"^    (uniform )?(\S+) ([A-Za-z_]\w*:[\w:]+)(?: = .*?)?(?:  # (.*))?$"
)


def parse_schema(text):
    classes = {}
    cur = None
    in_doc = False
    for line in text.splitlines():
        if not in_doc and cur is None:
            m = CLASS.match(line)
            if m:
                cur = {"name": m.group(1), "props": {}, "inherits": [],
                       "body": False}
                classes[cur["name"]] = cur
        if cur is not None and not cur["body"]:
            for m in INHERITS.finditer(line):
                cur["inherits"].append(m.group(1))
        count = line.count('"""')
        if cur is not None and not in_doc and cur["body"]:
            if line.startswith("}"):
                cur = None
            else:
                m = PROP.match(line)
                if m:
                    cur["props"][m.group(2)] = m.group(1)
        elif cur is not None and not in_doc and line.startswith("{"):
            cur["body"] = True
        if count % 2:
            in_doc = not in_doc
    return classes


def parse_ops(text):
    ops = {}
    cur = None
    for line in text.splitlines():
        m = OP_CLASS.match(line)
        if m:
            cur = {"class": m.group(1), "op": m.group(2), "ports": {},
                   "mode": m.group(4) or ""}
            ops[m.group(2)] = cur
            continue
        m = OP_PROP.match(line)
        if m and cur is not None:
            cur["ports"][m.group(3)] = (m.group(2), m.group(4) or "")
    return ops


def merged_props(classes, name, seen=None):
    seen = seen or set()
    if name in seen or name not in classes:
        return {}
    seen.add(name)
    out = {}
    for base in classes[name]["inherits"]:
        out.update(merged_props(classes, base, seen))
    out.update(classes[name]["props"])
    return out


def key(name):
    return name.split(":")[-1]


def norm(name):
    return re.sub(r"[^a-z0-9]", "", name.lower())


def find_class(op_class, classes, mode=""):
    if mode == "internal":
        return None, "internal"
    if mode and op_class in classes:
        return op_class, "declared"
    base = op_class
    candidates = []
    for prefix in ("RigExec",):
        for suffix in ("", "Mover", "Constraint", "Weight", "Frame", "API"):
            candidates.append(prefix + base + suffix)
            if base.endswith("s"):
                candidates.append(prefix + base[:-1] + suffix)
    for c in candidates:
        if c in classes:
            return c, "name"
    names = {norm(n): n for n in classes}
    best = difflib.get_close_matches(norm("RigExec" + base), list(names),
                                     n=1, cutoff=0.75)
    if best:
        return names[best[0]], "fuzzy"
    return None, "none"


FAMILY = {
    "float": "real", "double": "real", "half": "real",
    "int": "integer", "uint": "integer", "int64": "integer",
    "uint64": "integer", "uchar": "integer", "bool": "bool",
}


def type_class(op_type, schema_type):
    if schema_type == "rel":
        return "binding"
    o_arr, s_arr = op_type.endswith("[]"), schema_type.endswith("[]")
    o, s = op_type.rstrip("[]"), schema_type.rstrip("[]")
    if o == s:
        return "same" if o_arr == s_arr else "shape"
    if o.startswith("<"):
        return "other"
    if s == "token" and FAMILY.get(o) == "integer" and not o_arr:
        return "enum"
    role = {"point3f": "float3", "normal3f": "float3", "vector3f": "float3",
            "color3f": "float3", "point3d": "double3", "vector3d": "double3",
            "normal3d": "double3"}
    if role.get(s, s) == o and o_arr == s_arr:
        return "role"
    stem = lambda t: re.sub(r"[234]$", "", re.sub(r"(?<=[0-9])[fdh]$", "", t))
    if FAMILY.get(o) and FAMILY.get(o) == FAMILY.get(s):
        return "precision" if o_arr == s_arr else "shape"
    if re.sub(r"[fd]$", "", o) == re.sub(r"[fd]$", "", s) and o_arr == s_arr:
        return "precision"
    if stem(o) == stem(s):
        return "shape"
    return "other"


def report(ops, classes):
    lines = []
    rows = []
    used = {}
    for opname in sorted(ops):
        op = ops[opname]
        cls, how = find_class(op["class"], classes, op.get("mode", ""))
        row = {"op": opname, "cls": cls, "how": how, "schemaOnly": 0,
               "opOnly": 0, "binding": 0, "precision": 0, "shape": 0,
               "other": 0, "enum": 0, "role": 0, "matched": 0}
        lines.append("op %s -> %s" % (
            opname, "%s (%s)" % (cls, how) if cls else
            "internal" if how == "internal" else "no schema class"))
        if cls:
            used.setdefault(cls, []).append(opname)
            props = merged_props(classes, cls)
            by_key = {}
            for name, typ in props.items():
                by_key.setdefault(key(name), (name, typ))
            op_keys = {}
            for pname, (ptype, flags) in sorted(op["ports"].items()):
                op_keys.setdefault(key(pname), []).append(
                    (pname, ptype, flags))
            op_only, disagree = [], []
            for k, entries in sorted(op_keys.items()):
                if k not in by_key:
                    op_only.extend("%s %s" % (t, n) for n, t, f in entries)
                    continue
                sname, stype = by_key[k]
                row["matched"] += 1
                pname, ptype, flags = entries[0]
                cls_ = type_class(ptype.replace("uniform ", ""), stype)
                if cls_ != "same":
                    row[cls_] += 1
                    disagree.append("%s: op %s, schema %s %s [%s]" % (
                        k, ptype, stype, sname, cls_))
            schema_only = sorted(n for k, (n, t) in by_key.items()
                                 if k not in op_keys)
            row["opOnly"], row["schemaOnly"] = len(op_only), len(schema_only)
            lines.append("  schema properties the op lacks (%d): %s" % (
                len(schema_only), ", ".join(schema_only) or "-"))
            lines.append("  op ports the schema lacks (%d): %s" % (
                len(op_only), ", ".join(op_only) or "-"))
            lines.append("  type disagreements (%d):" % len(disagree))
            lines.extend("    " + d for d in disagree)
        else:
            row["opOnly"] = len(op["ports"])
            lines.append("  op ports (%d): %s" % (
                len(op["ports"]), ", ".join(sorted(op["ports"]))))
        rows.append(row)
        lines.append("")

    lines.append("summary")
    lines.append("%-24s %-32s %-6s %7s %6s %5s %5s %5s %5s %5s %5s" % (
        "op", "class", "match", "schOnly", "opOnly", "bind", "prec", "enum",
        "role", "shape", "other"))
    for r in rows:
        lines.append("%-24s %-32s %-6s %7d %6d %5d %5d %5d %5d %5d %5d" % (
            r["op"], r["cls"] or "-", r["how"], r["schemaOnly"], r["opOnly"],
            r["binding"], r["precision"], r["enum"], r["role"], r["shape"],
            r["other"]))
    mapped = [r for r in rows if r["cls"]]
    named = [r for r in mapped if r["how"] in ("name", "declared")]
    clean = [r for r in named if r["shape"] == 0 and r["other"] == 0]
    ports = sum(len(o["ports"]) for o in ops.values())
    matched = sum(r["matched"] for r in rows)
    lines.append("")
    lines.append("ops: %d" % len(rows))
    internal = [r for r in rows if r["how"] == "internal"]
    lines.append("ops with a class by name or declared: %d, by fuzzy name: "
                 "%d, internal: %d, none: %d"
                 % (len(named), len(mapped) - len(named), len(internal),
                    len(rows) - len(mapped) - len(internal)))
    lines.append("name-matched ops with no shape or other type disagreement "
                 "(precision and binding differences allowed): %d"
                 % len(clean))
    lines.append("op ports and payload fields: %d, found in the class by "
                 "name: %d" % (ports, matched))
    lines.append("type disagreements: binding %d, precision %d, enum %d, "
                 "role %d, shape %d, other %d" % tuple(
                     sum(r[k] for r in rows) for k in
                     ("binding", "precision", "enum", "role", "shape",
                      "other")))
    shared = {c: o for c, o in used.items() if len(o) > 1}
    for c, o in sorted(shared.items()):
        lines.append("class %s claimed by several ops: %s" % (c, ", ".join(o)))
    lines.append("")
    lines.append("schema classes no op corresponds to (%d of %d):" % (
        len([c for c in classes if c not in used]), len(classes)))
    lines.append("  " + ", ".join(sorted(c for c in classes if c not in used)))
    lines.append("ops no class corresponds to: " + (", ".join(
        r["op"] for r in rows if not r["cls"] and r["how"] != "internal")
        or "-"))
    lines.append("internal ops: " + (", ".join(
        r["op"] for r in rows if r["how"] == "internal") or "-"))
    return "\n".join(lines) + "\n"


def _closing(text, i):
    """Index just past the group opened at text[i], honouring strings."""
    pairs = {"(": ")", "[": "]", "{": "}"}
    stack = []
    n = len(text)
    while i < n:
        c = text[i]
        if text.startswith('"""', i):
            i = text.index('"""', i + 3) + 3
            continue
        if c == "#":
            i = text.find("\n", i)
            i = n if i < 0 else i
            continue
        if c in "\"'":
            j = i + 1
            while text[j] != c:
                j += 2 if text[j] == "\\" else 1
            i = j + 1
            continue
        if c in pairs:
            stack.append(pairs[c])
        elif stack and c == stack[-1]:
            stack.pop()
            if not stack:
                return i + 1
        i += 1
    raise ValueError("unbalanced group")


def _string(text):
    text = text.strip()
    if text.startswith('"""'):
        return text[3:-3]
    if text[:1] in "\"'":
        return text[1:-1]
    return text


def _metadata(text):
    """doc and allowedTokens from a metadata group's text (without parens)."""
    out = {}
    m = re.search(r'\bdoc\s*=\s*("""(?:.|\n)*?"""|"(?:[^"\\]|\\.)*")', text)
    if m:
        out["doc"] = _string(m.group(1))
    m = re.search(r"\ballowedTokens\s*=\s*\[", text)
    if m:
        start = m.end() - 1
        body = text[start + 1:_closing(text, start) - 1]
        out["tokens"] = re.findall(r'"([^"]*)"', body)
    m = re.search(r"\binherits\s*=\s*</?([A-Za-z0-9_]+)>", text)
    if m:
        out["base"] = m.group(1)
    return out


PROP_HEAD = re.compile(
    r"\s*((?:(?:custom|uniform)\s+)*)(" + USD_TYPE + r")\s+"
    r"([A-Za-z_][\w:.]*)\s*")
CLASS_HEAD = re.compile(r'^class\s+(?:\w+\s+)?"([^"]+)"[ \t]*', re.M)


def parse_schema_full(text):
    """Class name to {base, doc, props}; a prop is {type, uniform, fallback,
    tokens, doc}. Only a class's own properties are listed."""
    classes = {}
    for m in CLASS_HEAD.finditer(text):
        i = m.end()
        meta = {}
        if text[i] == "(":
            end = _closing(text, i)
            meta = _metadata(text[i + 1:end - 1])
            i = end
        start = text.index("{", i)
        end = _closing(text, start)
        body = text[start + 1:end - 1]
        props = {}
        j = 0
        while True:
            pm = PROP_HEAD.match(body, j)
            if not pm:
                nl = body.find("\n", j + 1)
                if nl < 0:
                    break
                j = nl
                continue
            prop = {"type": pm.group(2),
                    "uniform": "uniform" in pm.group(1).split(),
                    "fallback": None, "tokens": [], "doc": None}
            j = pm.end()
            if body.startswith("=", j):
                j += 1
                while body[j] in " \t":
                    j += 1
                if body[j] in "([":
                    k = _closing(body, j)
                elif body[j] == '"':
                    k = body.index('"', j + 1) + 1
                else:
                    k = j
                    while k < len(body) and body[k] not in " \t\n(":
                        k += 1
                prop["fallback"] = body[j:k]
                j = k
            while j < len(body) and body[j] in " \t":
                j += 1
            if body.startswith("(", j):
                k = _closing(body, j)
                md = _metadata(body[j + 1:k - 1])
                prop["tokens"] = md.get("tokens", [])
                prop["doc"] = md.get("doc")
                j = k
            props[pm.group(3)] = prop
        classes[m.group(1)] = {"base": meta.get("base"),
                               "doc": meta.get("doc"), "props": props}
    return classes


VALUE_TOKEN = re.compile(r'"[^"]*"|[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?'
                         r"|[A-Za-z_]\w*|[\[\]()]")


def same_value(a, b):
    if a is None or b is None:
        return a is b
    ta, tb = VALUE_TOKEN.findall(a), VALUE_TOKEN.findall(b)
    if len(ta) != len(tb):
        return False
    for x, y in zip(ta, tb):
        if x == y:
            continue
        try:
            if float(x) != float(y):
                return False
        except ValueError:
            return False
    return True


def same_doc(a, b):
    if a is None or b is None:
        return a is b
    return " ".join(a.split()) == " ".join(b.split())


def diff_property(where, mine, theirs):
    out = []
    if mine["type"] != theirs["type"]:
        out.append("%s: type %s, schema %s" % (where, mine["type"],
                                              theirs["type"]))
    if mine["uniform"] != theirs["uniform"]:
        out.append("%s: %suniform, schema %suniform" % (
            where, "" if mine["uniform"] else "not ",
            "" if theirs["uniform"] else "not "))
    if not same_value(mine["fallback"], theirs["fallback"]):
        out.append("%s: fallback %s, schema %s" % (
            where, mine["fallback"], theirs["fallback"]))
    if mine["tokens"] != theirs["tokens"]:
        out.append("%s: tokens %s, schema %s" % (
            where, mine["tokens"], theirs["tokens"]))
    if not same_doc(mine["doc"], theirs["doc"]):
        out.append("%s: doc %s, schema %s" % (
            where, "absent" if mine["doc"] is None else repr(mine["doc"]),
            "absent" if theirs["doc"] is None else repr(theirs["doc"])))
    return out


def inherited_props(classes, name, seen=None):
    seen = seen or set()
    if name in seen or name not in classes:
        return {}
    seen.add(name)
    out = inherited_props(classes, classes[name]["base"], seen)
    out.update(classes[name]["props"])
    return out


PROPERTY_KEYS = ("type", "uniform", "binding", "fallback", "tokens", "doc")


def disagreements(ops):
    """Where ops that declare one class describe it differently: the mode,
    and for Generated ops the class doc, base and property set; every
    property two ops both declare must match. The class is written once."""
    by_class = {}
    for op in sorted(ops, key=lambda o: o["op"]):
        schema = op["schema"]
        if schema and schema["mode"] != "internal":
            by_class.setdefault(schema["className"], []).append(op)
    out = []
    for name, group in sorted(by_class.items()):
        first = group[0]
        mine = {p["name"]: p for p in first["properties"]}
        for other in group[1:]:
            where = "class %s: ops %s and %s" % (name, first["op"], other["op"])
            a, b = first["schema"], other["schema"]
            if a["mode"] != b["mode"]:
                out.append("%s: mode %s, %s" % (where, a["mode"], b["mode"]))
                continue
            theirs = {p["name"]: p for p in other["properties"]}
            if a["mode"] == "generated":
                if not same_doc(a["doc"], b["doc"]):
                    out.append("%s: class doc differs" % where)
                if (a["base"] or None) != (b["base"] or None):
                    out.append("%s: base %s, %s" % (where, a["base"],
                                                    b["base"]))
                for prop in sorted(set(mine) ^ set(theirs)):
                    out.append("%s: only one declares %s" % (where, prop))
            for prop in sorted(set(mine) & set(theirs)):
                for k in PROPERTY_KEYS:
                    x, y = mine[prop][k], theirs[prop][k]
                    same = (same_doc(x, y) if k == "doc" else
                            same_value(x, y) if k == "fallback" else x == y)
                    if not same:
                        out.append("%s: %s %s %r, %r" % (where, prop, k, x, y))
    return out


def diff(ops, classes):
    """(lines, difference count) for the Generated and HandAuthored ops."""
    lines, count = [], 0
    skipped = {"internal": [], "undeclared": []}
    for op in sorted(ops, key=lambda o: o["op"]):
        schema = op["schema"]
        if not schema or schema["mode"] == "internal":
            skipped["internal" if schema else "undeclared"].append(op["op"])
            continue
        mode, name = schema["mode"], schema["className"]
        found = []
        for prop in op["properties"]:
            found.extend("%s: %s" % (prop["name"], e) for e in prop["errors"])
        cls = classes.get(name)
        if cls is None:
            found.append("class %s is not in the schema" % name)
        else:
            mine = {p["name"]: p for p in op["properties"]
                    if p["binding"] not in ("collection", "children")}
            if mode == "generated":
                theirs = cls["props"]
                if (schema["base"] or None) != cls["base"]:
                    found.append("base %s, schema %s" % (schema["base"],
                                                        cls["base"]))
                if not same_doc(schema["doc"], cls["doc"]):
                    found.append("class doc differs")
                for prop in sorted(set(theirs) - set(mine)):
                    found.append("%s: in the schema, not declared" % prop)
            else:
                theirs = inherited_props(classes, name)
            for prop in sorted(mine):
                if prop not in theirs:
                    found.append("%s: declared, not in the schema" % prop)
                    continue
                found.extend(diff_property(prop, mine[prop], theirs[prop]))
        lines.append("op %s -> %s (%s): %s" % (
            op["op"], name, mode,
            "%d difference(s)" % len(found) if found else "no differences"))
        lines.extend("  " + f for f in found)
        count += len(found)
    shared = disagreements(ops)
    lines.extend(shared)
    count += len(shared)
    lines.append("")
    lines.append("skipped internal ops: " + (", ".join(skipped["internal"])
                                             or "-"))
    lines.append("skipped ops without schema info: %d"
                 % len(skipped["undeclared"]))
    lines.append("differences: %d" % count)
    return "\n".join(lines) + "\n", count


def self_test():
    """Runs diff() over synthetic ops and classes; 0 when every case holds."""
    schema = parse_schema_full('''
class "Typed" { }
class "Thing" (
    doc = "A thing."
    inherits = </Typed>
)
{
    uniform token rigExec:mode = "a" (allowedTokens = ["a", "b"])
    float inputs:x = 1
}
''')

    def prop(name, type_, fallback, uniform=False, tokens=(), doc=None):
        return {"name": name, "type": type_, "uniform": uniform,
                "binding": "value", "fallback": fallback,
                "tokens": list(tokens), "doc": doc, "sources": [],
                "errors": []}

    def op(name, props, mode="generated", doc="A thing.", base="Typed"):
        return {"op": name, "version": 1, "properties": props,
                "schema": {"className": "Thing", "doc": doc, "base": base,
                           "mode": mode}}

    mode = prop("rigExec:mode", "token", '"a"', True, ("a", "b"))
    x = prop("inputs:x", "float", "1")
    cases = [
        ("agree", [op("A", [mode, x]), op("B", [mode, x])], None),
        ("fallback", [op("A", [mode, x]),
                      op("B", [mode, prop("inputs:x", "float", "2")])],
         "ops A and B: inputs:x fallback"),
        ("mode", [op("A", [mode, x]), op("B", [x], "handAuthored")],
         "mode generated, handAuthored"),
        ("set", [op("A", [mode, x]), op("B", [mode])],
         "only one declares inputs:x"),
        ("doc", [op("A", [mode, x]), op("B", [mode, x], doc="Other.")],
         "ops A and B: class doc differs"),
        ("binding", [op("A", [x], "handAuthored"),
                     op("B", [dict(x, binding="connection")],
                        "handAuthored")],
         "ops A and B: inputs:x binding"),
        ("subset", [op("A", [x], "handAuthored"),
                    op("B", [mode], "handAuthored")], None),
        ("missing", [op("A", [mode])], "inputs:x: in the schema"),
    ]
    failed = 0
    for name, ops, expect in cases:
        text, count = diff(ops, schema)
        ok = (count == 0) if expect is None else (count > 0 and expect in text)
        if not ok:
            failed += 1
            sys.stdout.write("self-test %s failed:\n%s" % (name, text))
    sys.stdout.write("self-test: %d of %d cases failed\n"
                     % (failed, len(cases)))
    return 1 if failed else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--tool")
    parser.add_argument("--ops")
    parser.add_argument("--schema")
    parser.add_argument("--out")
    parser.add_argument("--diff", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    if not args.schema:
        parser.error("--schema is required")
    if args.diff:
        if not args.tool:
            parser.error("--diff needs --tool")
        proc = subprocess.run([os.path.abspath(args.tool), "--json"],
                              capture_output=True, text=True)
        if proc.returncode != 0:
            sys.stderr.write(proc.stderr)
            return 1
        with open(args.schema, encoding="utf-8") as f:
            classes = parse_schema_full(f.read())
        text, count = diff(json.loads(proc.stdout)["ops"], classes)
        sys.stdout.write(text)
        if args.out:
            with open(args.out, "w", encoding="utf-8", newline="\n") as f:
                f.write(text)
        return 1 if count else 0
    if args.tool:
        proc = subprocess.run([os.path.abspath(args.tool)],
                              capture_output=True, text=True)
        if proc.returncode != 0:
            sys.stderr.write(proc.stderr)
            return 1
        ops_text = proc.stdout
    elif args.ops:
        ops_text = open(args.ops, encoding="utf-8").read()
    else:
        parser.error("give --tool or --ops")
    with open(args.schema, encoding="utf-8") as f:
        classes = parse_schema(f.read())
    ops = parse_ops(ops_text)
    if not ops or not classes:
        sys.stderr.write("no ops (%d) or no schema classes (%d) parsed\n"
                         % (len(ops), len(classes)))
        return 1
    text = report(ops, classes)
    sys.stdout.write(text)
    if args.out:
        with open(args.out, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
