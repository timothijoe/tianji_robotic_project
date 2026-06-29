"""Console entry point for the checkout-local trajectory demo."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from typing import Sequence

from twin_description import workspace_root


def main(argv: Sequence[str] | None = None) -> int:
    demo_path = workspace_root() / "examples" / "trajectory_demo.py"
    spec = importlib.util.spec_from_file_location("_twin_trajectory_demo", demo_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load trajectory demo: {demo_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return int(module.main(argv))


if __name__ == "__main__":
    raise SystemExit(main())
