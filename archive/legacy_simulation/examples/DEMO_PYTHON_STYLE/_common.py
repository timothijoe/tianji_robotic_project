"""Common helpers for SDK-style examples."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "src" / "twin_core"))
sys.path.insert(0, str(ROOT / "src" / "twin_description"))
sys.path.insert(0, str(ROOT / "src" / "twin_mujoco"))

from twin_control.sdk_compat import create_robot


def add_backend_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--backend", choices=("mujoco", "real"), default="mujoco")
    parser.add_argument("--arm", choices=("A", "B"), default="B")
    parser.add_argument("--robot-ip", default="mujoco")
    parser.add_argument("--sdk-root", type=Path, default=None)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-realtime", action="store_true")
    parser.add_argument("--control-hz", type=float, default=500.0)


def make_robot(args: argparse.Namespace):
    return create_robot(
        args.backend,
        arm=args.arm,
        viewer=args.viewer and not args.headless,
        realtime=not args.no_realtime,
        control_hz=args.control_hz,
        sdk_root=args.sdk_root,
    )
