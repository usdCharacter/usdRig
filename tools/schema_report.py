#!/usr/bin/env python3
"""Reports how far the schema generated from op registration is from
libs/rigExecSchema/schema.usda. Report mode only: nothing is written to the
repository and the exit status is 0 whenever a report was produced.

Inputs:
  --tool PATH    the rigExecGenSchema executable to run, or
  --ops PATH     the text it wrote (--tool wins)
  --schema PATH  schema.usda (a tolerant line parser; pxr is not imported)
  --out PATH     also write the report here
Output: per op the schema class it most plausibly corresponds to, the
properties each side lacks and the type disagreements, then a summary table
and the classes and ops without a counterpart.
"""
import argparse
import difflib
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
OP_CLASS = re.compile(r'^class "([^"]+)" \(  # op (\S+) v(\d+)')
OP_PROP = re.compile(
    r"^    (uniform )?(\S+) ((?:inputs|outputs|payload):\S+?)(?:  # (.*))?$"
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
            cur = {"class": m.group(1), "op": m.group(2), "ports": {}}
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


def find_class(op_class, classes):
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
        cls, how = find_class(op["class"], classes)
        row = {"op": opname, "cls": cls, "how": how, "schemaOnly": 0,
               "opOnly": 0, "binding": 0, "precision": 0, "shape": 0,
               "other": 0, "enum": 0, "role": 0, "matched": 0}
        lines.append("op %s -> %s" % (
            opname, "%s (%s)" % (cls, how) if cls else "no schema class"))
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
    named = [r for r in mapped if r["how"] == "name"]
    clean = [r for r in named if r["shape"] == 0 and r["other"] == 0]
    ports = sum(len(o["ports"]) for o in ops.values())
    matched = sum(r["matched"] for r in rows)
    lines.append("")
    lines.append("ops: %d" % len(rows))
    lines.append("ops with a class by name: %d, by fuzzy name: %d, none: %d"
                 % (len(named), len(mapped) - len(named),
                    len(rows) - len(mapped)))
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
        r["op"] for r in rows if not r["cls"]) or "-"))
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--tool")
    parser.add_argument("--ops")
    parser.add_argument("--schema", required=True)
    parser.add_argument("--out")
    args = parser.parse_args()
    if args.tool:
        proc = subprocess.run([args.tool], capture_output=True, text=True)
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
