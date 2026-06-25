from __future__ import annotations

import argparse
from pathlib import Path

from cook_description.models.converter import (
    ModelConversionError,
    convert_urdf_to_mjcf,
    package_roots_from_json,
)
from cook_description.paths import (
    DEFAULT_MJCF_PATH,
    DEFAULT_PACKAGE_NAME,
    DEFAULT_URDF_PATH,
    ROBOT_ASSET_ROOT,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Offline URDF-to-MJCF converter for robot visualization."
    )
    parser.add_argument("--urdf", default=str(DEFAULT_URDF_PATH), help="Input URDF.")
    parser.add_argument(
        "--output", default=str(DEFAULT_MJCF_PATH), help="Output MJCF/XML path."
    )
    parser.add_argument(
        "--package-root",
        action="append",
        default=[],
        metavar="PACKAGE=PATH",
        help="Package URI mapping. May be passed multiple times.",
    )
    parser.add_argument("--package-map", default="", help="JSON package map.")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite output.")
    parser.add_argument("--validate", action="store_true", help="Validate output.")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        package_roots = _default_package_roots()
        package_roots.update(_parse_package_roots(args.package_root))
        if args.package_map:
            package_roots.update(package_roots_from_json(args.package_map))
        output = convert_urdf_to_mjcf(
            urdf_path=args.urdf,
            output_path=args.output,
            package_roots=package_roots,
            overwrite=args.overwrite,
            validate=args.validate,
        )
    except ModelConversionError as exc:
        parser.exit(2, f"error: {exc}\n")
    print(f"wrote MJCF/XML: {output}")
    return 0


def _default_package_roots() -> dict[str, Path]:
    return {DEFAULT_PACKAGE_NAME: ROBOT_ASSET_ROOT}


def _parse_package_roots(items: list[str]) -> dict[str, Path]:
    result = {}
    for item in items:
        if "=" not in item:
            raise ModelConversionError(f"package root must use PACKAGE=PATH: {item}")
        package, path = item.split("=", 1)
        package = package.strip()
        if not package:
            raise ModelConversionError(f"package root has empty package name: {item}")
        result[package] = Path(path).expanduser()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
