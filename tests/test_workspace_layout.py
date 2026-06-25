from pathlib import Path
import shutil
import subprocess

import pytest


EXPECTED_PACKAGES = {
    "cook_core",
    "cook_description",
    "cook_mujoco",
    "cook_bringup",
}


def test_workspace_packages_live_under_src():
    root = Path(__file__).resolve().parents[1]

    assert not (root / "package.xml").exists()
    assert {
        path.parent.name for path in (root / "src").glob("cook_*/package.xml")
    } == EXPECTED_PACKAGES


def test_architecture_document_is_linked_from_readme():
    root = Path(__file__).resolve().parents[1]

    assert (root / "docs" / "ARCHITECTURE.md").is_file()
    assert "docs/ARCHITECTURE.md" in (root / "README.md").read_text()


def test_planner_guidance_is_documented():
    root = Path(__file__).resolve().parents[1]

    readme_text = (root / "README.md").read_text()
    architecture_text = (root / "docs" / "ARCHITECTURE.md").read_text()
    development_text = (root / "docs" / "DEVELOPMENT.md").read_text()
    planning_usage_text = (root / "docs" / "PLANNING_USAGE.md").read_text()

    assert "pip install ompl" in readme_text
    assert "docs/PLANNING_USAGE.md" in readme_text
    assert "docs/PLANNING_USAGE.md" in architecture_text
    assert "OmplJointPlanner" in architecture_text
    assert "MujocoCollisionChecker" in architecture_text
    assert "allow_initial_contacts" in architecture_text
    assert "OmplUnavailableError" in architecture_text
    assert "PlanRequest -> Planner.plan() -> PlanResult" in architecture_text
    assert "算法库内部状态、空间定义、碰撞检查对象不能泄漏" in architecture_text
    assert "OMPL 作为可选依赖" in development_text
    assert "PlanResult(success=False, message=...)" in development_text
    assert "fake/mock OMPL" in development_text
    assert "规划到执行使用指南" in planning_usage_text
    assert "RobotCommandPort -> ROS2 JointTrajectory -> MuJoCo controller" in planning_usage_text
    assert "python examples/planning_control_demo.py --planner ompl --backend ros2" in planning_usage_text
    assert "create_mujoco_ompl_planner" in planning_usage_text
    assert "site-packages/assets/robot/urdf/test1.urdf" in planning_usage_text
    assert "StateValidityCheckerFn" in planning_usage_text
    assert "space.allocState()" in planning_usage_text


def test_colcon_discovers_only_cook_packages():
    if shutil.which("colcon") is None:
        pytest.skip("colcon is not available")

    root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        ["colcon", "list", "--base-paths", "src"],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    )
    packages = {line.split()[0] for line in result.stdout.splitlines() if line.strip()}

    assert packages == EXPECTED_PACKAGES


def test_bringup_registers_teach_pendant_entrypoint_and_launch_argument():
    root = Path(__file__).resolve().parents[1]

    package_text = (root / "src" / "cook_bringup" / "package.xml").read_text()
    setup_text = (root / "src" / "cook_bringup" / "setup.py").read_text()
    launch_text = (
        root / "src" / "cook_bringup" / "launch" / "visualization.launch.py"
    ).read_text()
    rviz_text = (
        root / "src" / "cook_bringup" / "rviz" / "cook_visualization.rviz"
    ).read_text()

    assert "cook_teach_pendant = cook_bringup.ros.teach_pendant_node:main" in setup_text
    assert 'DeclareLaunchArgument("use_teach_pendant", default_value="true")' in launch_text
    assert 'executable="cook_teach_pendant"' in launch_text
    assert "<depend>geometry_msgs</depend>" in package_text
    assert "<depend>nav_msgs</depend>" in package_text
    assert "<depend>tf2_ros</depend>" in package_text
    assert 'DeclareLaunchArgument("publish_tcp_paths", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("demo_planner_type", default_value="linear")' in launch_text
    assert 'DeclareLaunchArgument("demo_collision_check", default_value="true")' in launch_text
    assert (
        'DeclareLaunchArgument("demo_allow_initial_contacts", default_value="true")'
        in launch_text
    )
    assert 'DeclareLaunchArgument("tcp_parent_body_name", default_value="Link7_L")' in launch_text
    assert 'DeclareLaunchArgument("tcp_frame_id", default_value="tcp_link")' in launch_text
    assert 'DeclareLaunchArgument("tcp_offset_xyz"' in launch_text
    assert 'DeclareLaunchArgument("tcp_offset_rpy"' in launch_text
    assert "/tcp/actual_path" in rviz_text
    assert "/tcp/predicted_path" in rviz_text
    assert "rviz_default_plugins/TF" in rviz_text
    assert "/end_effector/actual_path" not in rviz_text
    assert "/end_effector/predicted_path" not in rviz_text
    assert "publish_end_effector_paths" not in launch_text
