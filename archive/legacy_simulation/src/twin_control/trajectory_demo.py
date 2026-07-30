"""Package entry point for the trajectory demo.

The implementation lives in ``examples/trajectory_demo.py`` so it can still be
run directly from a checkout.  This wrapper makes ``python -m
twin_control.trajectory_demo`` and the ``twin-trajectory`` console script work
from the installed package metadata.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from typing import Sequence


def _load_examples_module():
    root = Path(__file__).resolve().parents[2]
    demo_path = root / "examples" / "trajectory_demo.py"
    spec = importlib.util.spec_from_file_location("_twin_examples_trajectory_demo", demo_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"failed to load trajectory demo from {demo_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def main(argv: Sequence[str] | None = None) -> int:
    return int(_load_examples_module().main(argv))


if __name__ == "__main__":
    raise SystemExit(main())
