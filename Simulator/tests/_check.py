"""Shared plain-script test helpers (same style as the repo's test_*.py)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

FAILURES: list[str] = []


def check(label, condition, detail=""):
    print(f"  {'ok  ' if condition else 'FAIL'} {label}"
          + (f" {detail}" if detail and not condition else ""))
    if not condition:
        FAILURES.append(label)


def finish(name):
    print()
    if FAILURES:
        print(f"{len(FAILURES)} FAILED: {FAILURES}")
        raise SystemExit(1)
    print(f"all {name} checks passed")
