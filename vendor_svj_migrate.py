#!/usr/bin/env python3
"""
vendor_svj_migrate.py - copy the SVJ suspension migration into svj_migrate/
==========================================================================

The converter writes the unified suspension (SVJ spec section 25) with the SVJ
standard's own migration tool. This script copies that tool (and the three
small modules it needs) from the unpacked specification into the repository,
so conversion works without the specification folder. Run it again whenever
the standard's migration changes:

    python vendor_svj_migrate.py            # from svj_spec/SVJ-standard-vehicle-json-main/tools
    python vendor_svj_migrate.py <tools dir>

Source: https://github.com/RFloEng/SVJ-standard-vehicle-json (Apache-2.0).
"""
import ast
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEST = HERE / "svj_migrate"
DEFAULT_TOOLS = HERE / "svj_spec" / "SVJ-standard-vehicle-json-main" / "tools"
HEADER = '"""{what}\nVendored from RFloEng/SVJ-standard-vehicle-json tools/ (Apache-2.0); regenerate with vendor_svj_migrate.py."""\n'


def naming_module(src: str) -> str:
    """The part-name vocabulary (constraints_check.py): tables and canonical_part_id()."""
    lines = src.splitlines()
    tree = ast.parse(src)
    start = next(i for i, l in enumerate(lines) if l.startswith("_SIDE_SUFFIX"))
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "canonical_part_id")
    body = "\n".join(lines[start:fn.end_lineno])
    return HEADER.format(what="Part-name vocabulary of the SVJ standard (spec 24.2).") + "import re\n\n" + body + "\n"


def relink(src: str) -> str:
    """Replace the try/except import block of the tools with package-relative imports."""
    src = re.sub(r"try:\s*# inside svj-py\n.*?\nexcept ImportError:.*?\n(?=\S)", "", src, count=1, flags=re.S) \
        if "# inside svj-py" in src else src
    return src


def main() -> None:
    tools = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_TOOLS
    if not (tools / "migrate_suspension.py").is_file():
        sys.exit(f"migrate_suspension.py not found in {tools}")
    DEST.mkdir(exist_ok=True)

    (DEST / "naming.py").write_text(naming_module((tools / "constraints_check.py").read_text(encoding="utf-8")),
                                    encoding="utf-8")

    mig = (tools / "migrate_suspension.py").read_text(encoding="utf-8")
    block = re.search(r"try:[^\n]*\n(?:[ \t]+.*\n)+except ImportError:[^\n]*\n(?:[ \t]+.*\n)+", mig)
    if not block:
        sys.exit("migrate_suspension.py: import block not recognised")
    mig = mig.replace(block.group(0),
                      "from . import naming as cc\n"
                      "from .multibody_check import infer_joint_types\n"
                      "from .suspension_view import _MAP_TYPE, _PREFIX, _hardpoint, _mean, _norm, _pos, _slug, _stations, _sub, quat_z_to\n")
    (DEST / "migrate_suspension.py").write_text(mig, encoding="utf-8")

    view = (tools / "suspension_view.py").read_text(encoding="utf-8")
    block = re.search(r"try:[^\n]*\n(?:[ \t]+.*\n)+except ImportError:[^\n]*\n(?:[ \t]+.*\n)+", view)
    if not block:
        sys.exit("suspension_view.py: import block not recognised")
    view = view.replace(block.group(0), "from .multibody_check import infer_joint_types\n")
    (DEST / "suspension_view.py").write_text(view, encoding="utf-8")

    (DEST / "multibody_check.py").write_text((tools / "multibody_check.py").read_text(encoding="utf-8"),
                                             encoding="utf-8")
    (DEST / "__init__.py").write_text(
        HEADER.format(what="The SVJ standard's suspension migration (corner section -> bodies, joints, elements).")
        + "from .migrate_suspension import migrate\n\n__all__ = [\"migrate\"]\n", encoding="utf-8")
    print(f"svj_migrate/ written from {tools}")


if __name__ == "__main__":
    main()
