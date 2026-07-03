# Showcase IK Cartesian Impedance Quick Start

Run commands from the project root:

```bash
cd /home/zhoutong/catkin_robotic_ws/cook_proj/receive_tianji
```

Use this `PYTHONPATH` for all commands below:

```bash
export PYTHONPATH=src:src/twin_core:src/twin_description:src/twin_mujoco
```

## 1. Vertical Down-Up Chop In Place

This keeps the tool at the same XY position and moves only along Z.

```bash
python3 examples/DEMO_PYTHON_STYLE/showcase_ik_cart_impedance.py \
  --backend mujoco \
  --viewer \
  --control-hz 250 \
  --dz-mm -20 \
  --hold-s 2.0 \
  --cycles 5
```

## 2. Vertical Cuts With Y Offset Between Cuts

This cuts straight down, retracts, shifts a little along Y, then cuts straight down again.

```bash
python3 examples/DEMO_PYTHON_STYLE/showcase_ik_cart_impedance.py \
  --backend mujoco \
  --viewer \
  --control-hz 250 \
  --dz-mm -20 \
  --hold-s 2.0 \
  --cycles 5 \
  --lateral \
  --lateral-mm 10
```

## 3. Show Target And Actual Trail Points In MuJoCo

Cyan points are the target TCP positions. Orange points are the actual TCP positions.

```bash
python3 examples/DEMO_PYTHON_STYLE/showcase_ik_cart_impedance.py \
  --backend mujoco \
  --viewer \
  --control-hz 250 \
  --dz-mm -20 \
  --hold-s 2.0 \
  --cycles 5 \
  --lateral \
  --lateral-mm 10 \
  --viewer-trace \
  --viewer-trace-stride 10
```

## 4. Save A PNG Trajectory Plot

This runs headless and writes a TCP Z target-vs-actual plot.

```bash
python3 examples/DEMO_PYTHON_STYLE/showcase_ik_cart_impedance.py \
  --backend mujoco \
  --headless \
  --control-hz 250 \
  --dz-mm -20 \
  --hold-s 2.0 \
  --cycles 5 \
  --lateral \
  --lateral-mm 10 \
  --plot /tmp/showcase_ik_cart_impedance_trace.png
```

Open the PNG with:

```bash
xdg-open /tmp/showcase_ik_cart_impedance_trace.png
```

## 5. IK To Joint Impedance Version

This uses the same TCP chop trajectory, but each target TCP pose is converted to target joints by IK, then tracked with joint impedance. This is closer to the vendor SDK joint-impedance flow. Robot self-collision is disabled by default in this demo because the current chopping home pose has self-contact in the MuJoCo collision geoms; use `--enable-self-collision` only for debugging that contact behavior.

```bash
python3 examples/DEMO_PYTHON_STYLE/showcase_ik_joint_impedance.py \
  --backend mujoco \
  --viewer \
  --control-hz 250 \
  --dz-mm -20 \
  --hold-s 2.0 \
  --cycles 5 \
  --lateral \
  --lateral-mm 10 \
  --viewer-trace \
  --viewer-trace-stride 10
```



For path tracking with joint impedance, use this mode. It limits target progress using the actual TCP projection plus lookahead, so the target does not run far ahead when the robot lags.

```bash
python3 examples/DEMO_PYTHON_STYLE/showcase_ik_joint_impedance.py \
  --backend mujoco \
  --viewer \
  --control-hz 250 \
  --dz-mm -40 \
  --hold-s 2.0 \
  --cycles 5 \
  --lateral \
  --lateral-mm 10 \
  --tracking-mode path \
  --path-speed-mm-s 80 \
  --lookahead-mm 8 \
  --path-tolerance-mm 6 \
  --velocity-ff 0.8 \
  --viewer-trace \
  --viewer-trace-stride 10
```

For larger cycle counts, keep the cumulative Y offset inside the reachable workspace. For example, `--dz-mm -40 --cycles 15 --lateral-mm 40` asks the TCP to move from Y=-100 mm to about Y=500 mm and fails IK around Y=180 mm, Z=290 mm. Use `--lateral-mm 10` or `--lateral-mm 15` for 15-cycle demos from this home pose.

Use these optional parameters to tune joint impedance stiffness and damping with SDK-style units:

```bash
python3 examples/DEMO_PYTHON_STYLE/showcase_ik_joint_impedance.py \
  --backend mujoco \
  --viewer \
  --control-hz 250 \
  --dz-mm -20 \
  --hold-s 2.0 \
  --cycles 5 \
  --lateral \
  --lateral-mm 10 \
  --joint-k 2,2,2,1.6,5,5,5 \
  --joint-d 0.3,0.3,0.3,0.2,0.5,0.5,0.5 \
  --velocity-ff 0.8
```

## Parameter Notes

- `--cycles`: number of chop cycles.
- `--hold-s`: duration of one chop cycle in seconds.
- `--dz-mm`: Z movement depth. Negative values cut downward.
- `--lateral`: enable Y offset between consecutive vertical cuts.
- `--lateral-mm`: Y offset distance between consecutive cuts, in millimetres.
- `--viewer-trace`: draw target and actual TCP trail points in MuJoCo.
- `--viewer-trace-stride`: draw one point every N control steps. Larger values draw fewer points and run faster.
- `--joint-k`: joint impedance stiffness for `showcase_ik_joint_impedance.py`, 7 comma-separated SDK-style values.
- `--joint-d`: joint impedance damping for `showcase_ik_joint_impedance.py`, 7 comma-separated SDK-style values.
- `--enable-self-collision`: keep robot self-collision enabled in `showcase_ik_joint_impedance.py` for debugging.
- `--velocity-ff`: joint target velocity feedforward scale for `showcase_ik_joint_impedance.py`; default is `0.8`.
- `--tracking-mode`: `trajectory` uses time-indexed targets; `path` uses actual TCP progress plus lookahead.
- `--path-speed-mm-s`: desired path progress speed for joint-impedance path tracking.
- `--lookahead-mm`: maximum target distance ahead of the actual TCP projection in path mode.
- `--path-tolerance-mm`: distance to a segment endpoint before path mode advances to the next segment.
