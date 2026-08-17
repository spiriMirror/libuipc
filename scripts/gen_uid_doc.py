"""Generate official UID tables for the specification documents.

Scans the UID registration blocks in:
  - src/constitution/*.cpp                (REGISTER_CONSTITUTION_UIDS)
  - src/geometry/implicit_geometries/*.cpp (REGISTER_IMPLICIT_GEOMETRY_UIDS)

resolves file-local `constexpr U64 ... = N` constants, and rewrites the
AUTO-GENERATED section of:
  - docs/specification/constitution_uid.md
  - docs/specification/implicit_geometry_uid.md

Usage:
    python scripts/gen_uid_doc.py           # rewrite the tables in place
    python scripts/gen_uid_doc.py --check   # exit 1 if out of date (for CI)
"""

import argparse
import re
import sys
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent.parent

BEGIN_MARKER = "<!-- AUTO-GENERATED UID TABLE: BEGIN (scripts/gen_uid_doc.py) -->"
END_MARKER = "<!-- AUTO-GENERATED UID TABLE: END -->"

# [static] constexpr U64 HookeanSpringUID = 12[ull];
CONST_RE = re.compile(r"constexpr\s+U64\s+(\w+)\s*=\s*(\d+)\s*(?:ull?)?\s*;")
# auto affine_body = string{builtin::AffineBody};  (type alias variables)
TYPE_VAR_RE = re.compile(r"(?:auto|string)\s+(\w+)\s*=\s*(?:std::)?string\{\s*builtin::(\w+)\s*\}")
# UIDInfo{ .uid = ..., .name = "...", .type = ... }  (designated initializers)
UIDINFO_RE = re.compile(r"UIDInfo\s*\{(.*?)\}", re.S)
UID_RE = re.compile(r"\.uid\s*=\s*([^,}]+)")
NAME_RE = re.compile(r"\.name\s*=\s*\"([^\"]+)\"")
TYPE_RE = re.compile(r"\.type\s*=\s*([^,}]+)")
BUILTIN_TYPE_RE = re.compile(r"builtin::(\w+)")


def collect_uids(src_dir: Path) -> list[dict]:
    entries = []
    for cpp in sorted(src_dir.glob("*.cpp")):
        text = cpp.read_text(encoding="utf-8")
        consts = {m.group(1): int(m.group(2)) for m in CONST_RE.finditer(text)}
        type_vars = {m.group(1): m.group(2) for m in TYPE_VAR_RE.finditer(text)}
        for block in UIDINFO_RE.finditer(text):
            body = block.group(1)
            uid_m, name_m, type_m = (
                UID_RE.search(body),
                NAME_RE.search(body),
                TYPE_RE.search(body),
            )
            if not (uid_m and name_m and type_m):
                continue
            uid_expr = uid_m.group(1).strip()
            uid = consts.get(uid_expr)
            if uid is None:
                num = re.fullmatch(r"(\d+)\s*(?:ull?)?", uid_expr)
                if num:
                    uid = int(num.group(1))
            if uid is None:
                print(f"[warn] unresolved uid expression `{uid_expr}` in {cpp.name}")
                continue
            type_expr = type_m.group(1).strip()
            builtin_m = BUILTIN_TYPE_RE.search(type_expr)
            # tolerate truncation at the inner `}` of e.g. `string{Constraint}}`
            brace_m = re.search(r"\{\s*(\w+)", type_expr)
            if builtin_m:
                type_name = builtin_m.group(1)
            elif type_expr in type_vars:
                type_name = type_vars[type_expr]
            elif brace_m:
                type_name = brace_m.group(1)
            else:
                type_name = type_expr
            entries.append(
                {
                    "uid": uid,
                    "name": name_m.group(1),
                    "type": type_name,
                    "source": cpp.relative_to(PROJECT_DIR).as_posix(),
                }
            )
    entries.sort(key=lambda e: (e["uid"], e["name"]))
    return entries


def render_table(entries: list[dict]) -> str:
    lines = [
        BEGIN_MARKER,
        "",
        "| UID | Name | Type | Source |",
        "| --- | ---- | ---- | ------ |",
    ]
    for e in entries:
        lines.append(f"| {e['uid']} | {e['name']} | {e['type']} | `{e['source']}` |")
    lines += ["", END_MARKER]
    return "\n".join(lines)


def update_doc(doc_path: Path, table: str, check: bool) -> bool:
    """Insert or replace the auto-generated section. Returns True if up to date."""
    text = doc_path.read_text(encoding="utf-8")
    if BEGIN_MARKER in text and END_MARKER in text:
        pattern = re.compile(
            re.escape(BEGIN_MARKER) + r".*?" + re.escape(END_MARKER), re.S
        )
        new_text = pattern.sub(lambda _: table, text)
    else:
        new_text = text.rstrip() + "\n\n## Official UID List\n\n" + table + "\n"
    if new_text == text:
        return True
    if check:
        print(f"[out of date] {doc_path}")
        return False
    doc_path.write_text(new_text, encoding="utf-8")
    print(f"[updated] {doc_path}")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="do not write; exit 1 if any document is out of date",
    )
    args = parser.parse_args()

    ok = True
    ok &= update_doc(
        PROJECT_DIR / "docs/specification/constitution_uid.md",
        render_table(collect_uids(PROJECT_DIR / "src/constitution")),
        args.check,
    )
    ok &= update_doc(
        PROJECT_DIR / "docs/specification/implicit_geometry_uid.md",
        render_table(collect_uids(PROJECT_DIR / "src/geometry/implicit_geometries")),
        args.check,
    )
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
