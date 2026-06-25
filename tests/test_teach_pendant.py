from cook_core.interfaces import JointStateData
from cook_core.teaching import TeachPendantModel


def test_teach_pendant_builds_limited_joint_trajectory():
    model = TeachPendantModel(
        joint_names=("joint_1", "joint_2"),
        joint_limits={
            "joint_1": (-1.0, 1.0),
            "joint_2": (None, 2.0),
        },
    )
    model.update_current_state(
        JointStateData(names=("joint_1", "joint_2"), positions=(0.2, 0.4))
    )

    trajectory = model.build_trajectory(
        target_positions={"joint_1": 3.0, "joint_2": 1.5},
        duration_sec=2.5,
    )

    assert trajectory.joint_names == ("joint_1", "joint_2")
    assert trajectory.source == "teach_pendant"
    assert trajectory.points[0].positions == {"joint_1": 0.2, "joint_2": 0.4}
    assert trajectory.points[0].time_from_start_sec == 0.0
    assert trajectory.points[1].positions == {"joint_1": 1.0, "joint_2": 1.5}
    assert trajectory.points[1].time_from_start_sec == 2.5


def test_teach_pendant_ignores_unrelated_joint_state_names():
    model = TeachPendantModel(joint_names=("joint_1", "joint_2"))
    model.update_current_state(
        JointStateData(names=("joint_2", "joint_3"), positions=(0.8, 9.0))
    )

    trajectory = model.build_trajectory(
        target_positions={"joint_1": 0.1, "joint_2": 0.2},
        duration_sec=1.0,
    )

    assert trajectory.points[0].positions == {"joint_1": 0.0, "joint_2": 0.8}
