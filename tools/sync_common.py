#!/usr/bin/env python3
"""
Sync the shared core in `common/` into each Arduino sketch folder.

Why this exists
---------------
The Arduino build only compiles the sources that live inside a sketch folder,
so a sketch cannot `#include` a header from a sibling directory. The shared
logic therefore has to be *copied* next to each `.ino`. Copies rot, so this
script is the only supported way to change them:

    edit common/*.h / common/*.cpp
    python3 tools/sync_common.py            # copies them into the sketches
    python3 tools/sync_common.py --check    # CI: are the copies up to date?

The copies are byte-identical to the originals, which is what makes `--check`
a reliable guard: if someone edits a copy inside a sketch folder, CI fails and
points at the file that should have been edited instead.

Usage
-----
    tools/sync_common.py             # sync and report what changed
    tools/sync_common.py --check     # exit 1 if any copy is stale (CI mode)
    tools/sync_common.py --dry-run   # show what would change
"""

from __future__ import annotations

import argparse
import hashlib
import pathlib
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
COMMON_DIR = REPO_ROOT / "common"
SKETCH_DIRS = ["hand_transmit", "hand_receive"]
SUFFIXES = (".h", ".cpp")


def shared_sources() -> list[pathlib.Path]:
    """Every shared source file, in a stable order."""
    if not COMMON_DIR.is_dir():
        sys.exit(f"error: {COMMON_DIR} does not exist")
    return sorted(p for p in COMMON_DIR.iterdir() if p.suffix in SUFFIXES)


def digest(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def same_content(a: pathlib.Path, b: pathlib.Path) -> bool:
    return b.is_file() and digest(a) == digest(b)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument(
        "--check",
        action="store_true",
        help="do not write anything; exit 1 if the copies are stale",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="report what would change without writing",
    )
    args = parser.parse_args()

    sources = shared_sources()
    if not sources:
        sys.exit(f"error: no shared sources found in {COMMON_DIR}")

    stale: list[str] = []
    copied = 0

    for sketch_name in SKETCH_DIRS:
        sketch_dir = REPO_ROOT / sketch_name
        if not sketch_dir.is_dir():
            sys.exit(f"error: sketch folder {sketch_dir} does not exist")

        for src in sources:
            dst = sketch_dir / src.name
            if same_content(src, dst):
                continue
            stale.append(f"{dst.relative_to(REPO_ROOT)}")
            if args.check or args.dry_run:
                continue
            dst.write_bytes(src.read_bytes())
            copied += 1

    if args.check:
        if stale:
            print("shared core is OUT OF SYNC:", file=sys.stderr)
            for item in stale:
                print(f"  stale: {item}", file=sys.stderr)
            print(
                "\nEdit common/ and run `python3 tools/sync_common.py`, "
                "never the copies.",
                file=sys.stderr,
            )
            return 1
        print(
            f"shared core in sync ({len(sources)} files x "
            f"{len(SKETCH_DIRS)} sketches)"
        )
        return 0

    if args.dry_run:
        if stale:
            print("would update:")
            for item in stale:
                print(f"  {item}")
        else:
            print("nothing to do")
        return 0

    if copied:
        print(f"copied {copied} file(s) from common/ into the sketch folders")
    else:
        print("sketch folders already up to date")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
