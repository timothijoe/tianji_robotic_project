#!/usr/bin/env python3
"""Real robot sampled IK position-mode chopping entrypoint.

This wrapper follows the MuJoCo chopping demo more closely than the MOVLA
planned-Cartesian wrapper: it forces the implementation to generate a TCP target
at every control tick, solve IK, and send joint position commands.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from real_robot_debug import real_ik_cart_impedance_lateral as impl


def main(argv: list[str] | None = None) -> int:
    args = list(argv) if argv is not None else sys.argv[1:]
    return impl.main([*args, "--command-mode", "position"])


if __name__ == "__main__":
    raise SystemExit(main())
