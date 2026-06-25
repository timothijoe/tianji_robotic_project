from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET


class RobotModelError(RuntimeError):
    pass


@dataclass(frozen=True)
class JointDefinition:
    name: str
    joint_type: str
    parent_link: str
    child_link: str
    lower: float | None = None
    upper: float | None = None
    effort: float | None = None
    velocity: float | None = None


@dataclass(frozen=True)
class RobotDefinition:
    name: str
    links: tuple[str, ...]
    joints: tuple[JointDefinition, ...]

    @property
    def movable_joint_names(self) -> tuple[str, ...]:
        return tuple(
            joint.name
            for joint in self.joints
            if joint.joint_type in {"revolute", "continuous", "prismatic"}
        )

    @property
    def joint_limits(self) -> dict[str, tuple[float | None, float | None]]:
        return {joint.name: (joint.lower, joint.upper) for joint in self.joints}


def load_robot_definition(path: str | Path) -> RobotDefinition:
    urdf_path = Path(path).expanduser()
    if not urdf_path.is_file():
        raise RobotModelError(f"URDF file does not exist: {urdf_path}")
    try:
        root = ET.parse(urdf_path).getroot()
    except ET.ParseError as exc:
        raise RobotModelError(f"failed to parse URDF: {urdf_path}") from exc

    if root.tag != "robot":
        raise RobotModelError(f"URDF root must be <robot>: {urdf_path}")

    links = tuple(
        _required_attr(link, "name", element_name="link") for link in root.findall("link")
    )
    joints = tuple(_parse_joint(joint) for joint in root.findall("joint"))
    return RobotDefinition(
        name=root.attrib.get("name", urdf_path.stem),
        links=links,
        joints=joints,
    )


def _parse_joint(element: ET.Element) -> JointDefinition:
    limit = element.find("limit")
    return JointDefinition(
        name=_required_attr(element, "name", element_name="joint"),
        joint_type=element.attrib.get("type", "").strip(),
        parent_link=_required_child_attr(element, "parent", "link"),
        child_link=_required_child_attr(element, "child", "link"),
        lower=_optional_float(limit, "lower"),
        upper=_optional_float(limit, "upper"),
        effort=_optional_float(limit, "effort"),
        velocity=_optional_float(limit, "velocity"),
    )


def _required_attr(element: ET.Element, name: str, *, element_name: str) -> str:
    value = element.attrib.get(name, "").strip()
    if not value:
        raise RobotModelError(f"{element_name} is missing required attribute: {name}")
    return value


def _required_child_attr(element: ET.Element, child_name: str, attr_name: str) -> str:
    child = element.find(child_name)
    if child is None:
        raise RobotModelError(
            f"joint {element.attrib.get('name', '<unknown>')} is missing <{child_name}>"
        )
    return _required_attr(child, attr_name, element_name=child_name)


def _optional_float(element: ET.Element | None, name: str) -> float | None:
    if element is None or name not in element.attrib:
        return None
    try:
        return float(element.attrib[name])
    except ValueError as exc:
        raise RobotModelError(f"invalid numeric joint limit {name}") from exc
