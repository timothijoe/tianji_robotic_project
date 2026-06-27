# twin_joint_ws

Pure MuJoCo phase-one workspace for a dual-arm Marvin model. The source robot
asset is kept under `MarvinCCS/marvin_final_fixed.xml`; phase-one task geometry
lives in `src/twin_description/twin_description/assets/robot/mujoco/right_chopping_scene.xml`.

## Setup

```bash
python3 -m pip install -e .[test]
```

## Test

```bash
pytest -v
```

## Headless right-arm chopping

```bash
twin-chop --cycles 3 --headless --log /tmp/twin_right_chop.csv
```

## Visual right-arm chopping

```bash
twin-chop --cycles 3 --viewer --log /tmp/twin_right_chop.csv
```

The first phase is pure MuJoCo. ROS2 nodes and launch files are intentionally
out of scope.
