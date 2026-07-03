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

## Parameter Notes

- `--cycles`: number of chop cycles.
- `--hold-s`: duration of one chop cycle in seconds.
- `--dz-mm`: Z movement depth. Negative values cut downward.
- `--lateral`: enable Y offset between consecutive vertical cuts.
- `--lateral-mm`: Y offset distance between consecutive cuts, in millimetres.
- `--viewer-trace`: draw target and actual TCP trail points in MuJoCo.
- `--viewer-trace-stride`: draw one point every N control steps. Larger values draw fewer points and run faster.
