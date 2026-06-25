from __future__ import annotations

import importlib
import json
import os
import shutil
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET


class ModelConversionError(RuntimeError):
    pass


def require_mujoco():
    try:
        return importlib.import_module("mujoco")
    except ImportError as exc:
        raise ModelConversionError(
            "MuJoCo is required for offline URDF-to-MJCF conversion. "
            "Install it with `pip install mujoco`."
        ) from exc


def convert_urdf_to_mjcf(
    *,
    urdf_path: str | Path,
    output_path: str | Path,
    package_roots: dict[str, str | Path] | None = None,
    mesh_dirs: list[str | Path] | None = None,
    overwrite: bool = False,
    validate: bool = False,
) -> Path:
    urdf = Path(urdf_path).expanduser()
    output = Path(output_path).expanduser()
    if urdf.suffix.lower() != ".urdf":
        raise ModelConversionError(f"input must be a URDF file: {urdf}")
    if not urdf.is_file():
        raise ModelConversionError(f"URDF file does not exist: {urdf}")
    if output.suffix.lower() not in {".xml", ".mjcf"}:
        raise ModelConversionError(f"output must be an MJCF/XML file: {output}")
    if output.exists() and not overwrite:
        raise ModelConversionError(f"output already exists, pass overwrite=True: {output}")

    resolved_urdf = resolve_urdf_asset_paths(
        urdf.read_text(),
        urdf_dir=urdf.parent,
        package_roots=package_roots,
        mesh_dirs=mesh_dirs,
    )

    mujoco = require_mujoco()
    with tempfile.TemporaryDirectory(prefix="robot_urdf_to_mjcf_") as temp_dir:
        temp_root = Path(temp_dir)
        staged_urdf, staged_meshes = stage_meshes_for_mujoco(
            resolved_urdf,
            staging_dir=temp_root,
        )
        staged_urdf_path = temp_root / urdf.name
        staged_urdf_path.write_text(staged_urdf)
        try:
            model = mujoco.MjModel.from_xml_path(str(staged_urdf_path))
        except Exception as exc:
            raise ModelConversionError(
                f"failed to load offline-resolved URDF with MuJoCo: {exc}"
            ) from exc

        output.parent.mkdir(parents=True, exist_ok=True)
        try:
            mujoco.mj_saveLastXML(str(output), model)
        except Exception as exc:
            raise ModelConversionError(f"failed to write MJCF/XML: {output}") from exc
        restore_mesh_paths(output, staged_meshes, relative_to=output.parent)

    if validate:
        try:
            mujoco.MjModel.from_xml_path(str(output))
        except Exception as exc:
            raise ModelConversionError(f"generated MJCF failed validation: {exc}") from exc
    return output


def resolve_urdf_asset_paths(
    urdf_text: str,
    *,
    urdf_dir: str | Path,
    package_roots: dict[str, str | Path] | None = None,
    mesh_dirs: list[str | Path] | None = None,
) -> str:
    root = ET.fromstring(urdf_text)
    lookup = {
        name: Path(path).expanduser().resolve()
        for name, path in (package_roots or {}).items()
    }
    roots = [Path(urdf_dir).expanduser().resolve()]
    roots.extend(Path(path).expanduser().resolve() for path in (mesh_dirs or []))

    changed = False
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if not filename:
            continue
        resolved = _resolve_mesh_filename(filename, package_roots=lookup, mesh_roots=roots)
        if resolved != filename:
            mesh.set("filename", resolved)
            changed = True
    if not changed:
        return urdf_text
    return ET.tostring(root, encoding="unicode")


def stage_meshes_for_mujoco(
    urdf_text: str,
    *,
    staging_dir: str | Path,
) -> tuple[str, dict[str, Path]]:
    root = ET.fromstring(urdf_text)
    staging_root = Path(staging_dir)
    staging_root.mkdir(parents=True, exist_ok=True)
    source_to_staged: dict[Path, str] = {}
    staged_to_source: dict[str, Path] = {}
    changed = False

    for index, mesh in enumerate(root.findall(".//mesh")):
        filename = mesh.get("filename")
        if not filename:
            continue
        source = Path(filename).expanduser()
        if not source.is_absolute():
            source = (staging_root / source).resolve()
        if not source.is_file():
            raise ModelConversionError(f"resolved mesh file does not exist: {source}")
        staged_name = source_to_staged.get(source)
        if staged_name is None:
            staged_name = f"mesh_{index:04d}{source.suffix.lower()}"
            shutil.copy2(source, staging_root / staged_name)
            source_to_staged[source] = staged_name
            staged_to_source[staged_name] = source
        mesh.set("filename", staged_name)
        changed = True

    if not changed:
        return urdf_text, staged_to_source
    return ET.tostring(root, encoding="unicode"), staged_to_source


def restore_mesh_paths(
    mjcf_path: str | Path,
    staged_meshes: dict[str, Path],
    *,
    relative_to: str | Path | None = None,
) -> None:
    if not staged_meshes:
        return
    path = Path(mjcf_path)
    tree = ET.parse(path)
    root = tree.getroot()
    base = None if relative_to is None else Path(relative_to).resolve()
    changed = False

    for mesh in root.findall(".//mesh"):
        file_value = mesh.get("file")
        if not file_value:
            continue
        source = staged_meshes.get(Path(file_value).name)
        if source is None:
            continue
        restored = _portable_path(source.resolve(), base=base)
        if file_value != restored:
            mesh.set("file", restored)
            changed = True

    if changed:
        tree.write(path, encoding="unicode")


def package_roots_from_json(path: str | Path) -> dict[str, Path]:
    map_path = Path(path).expanduser()
    try:
        raw = json.loads(map_path.read_text())
    except json.JSONDecodeError as exc:
        raise ModelConversionError(f"invalid package map JSON: {map_path}") from exc
    if not isinstance(raw, dict):
        raise ModelConversionError(f"package map must be a JSON object: {map_path}")
    return {str(name): Path(value).expanduser() for name, value in raw.items()}


def _resolve_mesh_filename(
    filename: str,
    *,
    package_roots: dict[str, Path],
    mesh_roots: list[Path],
) -> str:
    if filename.startswith("package://"):
        rest = filename[len("package://") :]
        package, _, relative = rest.partition("/")
        if not package or not relative:
            raise ModelConversionError(f"invalid package URI: {filename}")
        package_root = package_roots.get(package)
        if package_root is None:
            raise ModelConversionError(
                f"unresolved package URI {filename}; provide a package root"
            )
        candidate = package_root / relative
        if not candidate.is_file():
            raise ModelConversionError(f"package URI mesh does not exist: {filename}")
        return str(candidate.resolve())

    path = Path(filename).expanduser()
    if path.is_absolute():
        if not path.is_file():
            raise ModelConversionError(f"mesh file does not exist: {path}")
        return str(path.resolve())

    for root in mesh_roots:
        candidate = root / filename
        if candidate.is_file():
            return str(candidate.resolve())
    searched = ", ".join(str(root) for root in mesh_roots)
    raise ModelConversionError(
        f"relative mesh file does not exist: {filename}; searched: {searched}"
    )


def _portable_path(path: Path, *, base: Path | None) -> str:
    if base is None:
        return str(path)
    return Path(os.path.relpath(path, start=base)).as_posix()
