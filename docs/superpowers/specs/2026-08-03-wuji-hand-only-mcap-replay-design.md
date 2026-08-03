# Wuji Hand-Only MCAP MuJoCo Replay Design

Date: 2026-08-03

## Goal

Make `tianji-robot sim wuji-replay` a pure Wuji Hand validation path. The
MuJoCo model must contain only the official left Wuji Hand, never the Tianji or
Marvin arms. Existing arm-plus-hand simulations under `twin-sim` remain
unchanged.

## Ownership and repository boundary

The exact official hand-only subset is vendored into our repository:

```text
tianji_robotic_project/
└── robot_assets/mujoco/wuji_hand_standalone/
    ├── LICENSE
    ├── README.md
    ├── mjcf/left.xml
    └── meshes/left/*.STL
```

This subset consists of one 24 KB MJCF, 52 left-hand meshes (about 3.17 MB),
the MIT license and a provenance record containing source repository and commit
`63d2785932eb5b91523e44222f5d0bcffbce1e74`. It excludes the right hand, URDF,
RViz, ROS launch files, demo video, sample trajectory and official Python
script. The adjacent official repositories remain unchanged as upgrade
references. All non-official integration code, validation, replay, tests, CLI
and documentation remain under `tianji_robotic_project`.

## Asset path contract

The runtime model path is derived from the installed source package's project
asset resolver, never from the current working directory:

```text
<project-root>/robot_assets/mujoco/wuji_hand_standalone/mjcf/left.xml
```

A missing asset raises a concise error showing the resolved project path.
There is no environment override and no silent fallback to the arm scene or
the adjacent official checkout. Updating assets is an explicit, reviewed
vendor operation rather than a runtime dependency.

## Hand-only backend

Replace the `RightArmRobot`-based implementation of
`tianji_robotics.simulation.wuji_hand.MujocoWujiHand` with a backend that:

- loads the official left MJCF with `mujoco.MjModel.from_xml_path`;
- requires exactly the canonical 20 `fingerN_jointM` joints and actuators;
- maps the project-facing canonical names `left_fingerN_jointM` to the
  official names without changing trajectory order;
- derives safe command ranges from the intersection of joint and actuator
  ranges;
- commands `data.ctrl`, steps with `mujoco.mj_step`, and reads `data.qpos`;
- contains no import of `twin_sim.robot` or `SimWujiHand`;
- uses the official demo camera (`lookat [0,0,0.05]`, distance `0.5`, azimuth
  `180`, elevation `-20`) when Viewer is enabled;
- closes Viewer resources idempotently.

The existing backend-neutral replay and trajectory validation contracts remain
unchanged. Headless and Viewer modes use the same model and control mapping.

## CLI and data flow

The public command remains stable:

```bash
.venv-wuji-teleop/bin/tianji-robot sim wuji-replay SOURCE [--headless]
```

Data flow remains:

```text
Studio right-glove MCAP
  -> right-to-left skeleton mirror
  -> official wuji-sdk retargeting
  -> validated 20-joint trajectory
  -> official hand-only MuJoCo model
```

Optional NPZ and JointState MCAP outputs remain byte-format compatible. No
physical Wuji runtime is imported or connected.

## Failure behavior and safety

Before sending the first MuJoCo command, the implementation validates model
identity, all joint/actuator mappings, finite ranges, the whole trajectory and
maximum per-frame step. Missing or incompatible official assets fail before
Viewer launch. The hand-only command exposes no arm or physical-hardware
options.

## Verification

Automated tests must prove:

- the vendored asset path resolves independently of the current directory;
- missing assets fail clearly;
- the loaded model has `nq == nv == nu == 20`;
- it contains the Wuji palm and all 20 hand joints;
- it contains no Tianji/Marvin arm bodies, joints or actuators;
- command/read/step/close behavior satisfies the backend contract;
- the real 1676-frame recording completes Headless replay;
- Viewer initialization receives the official hand camera values;
- existing twin-arm and guarded-chop behavior remains unchanged.

Manual acceptance is one command: the Viewer must show only the left Wuji Hand
performing the recorded motion, with no mechanical arm visible.

## Deferred interfaces

Arm-mounted Wuji replay remains a separate future simulation backend rather
than a flag on the hand-only command. Physical SDK and ROS 2 adapters retain
their existing interfaces and are not implemented or connected by this work.
