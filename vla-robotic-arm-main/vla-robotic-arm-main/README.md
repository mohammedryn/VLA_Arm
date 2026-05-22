# VLA Robotic Arm

## Project Overview

A 4-DOF robotic arm controlled by a Vision-Language-Action (VLA) policy that
accepts natural-language instructions ("pick up the red cube") and executes
pick-place, stacking, and sorting tasks on a 60×60 cm tabletop workspace.
Perception runs on a Raspberry Pi 5 at 8 Hz using a YOLO cube detector, a
monocular + ToF pose estimator, and a flan-t5-small language encoder; low-level
control runs on a Teensy 4.1 at 50 Hz. Behaviour is decomposed hierarchically
into four learned skills — **REACH → GRASP → LIFT → PLACE** — each with its own
termination condition detected via IMU contact sensing and servo load feedback.

---

## Hardware

| Component | Details |
|---|---|
| **Arm** | 4-DOF, 5× STS3215 serial-bus servos, 60 mm riser base, d₁ = 125 mm |
| **Links** | a₂ = 130 mm · a₃ = 190 mm (DH parameterisation) |
| **Joint limits** | J0 ±150° · J1 −30°…+60° · J2 −120°…+30° · J3 0°…+90° |
| **Teensy 4.1** | 50 Hz servo control loop, telemetry at 2 Mbaud over USB |
| **Raspberry Pi 5 8 GB** | 8 Hz VLA inference, PyTorch / Transformers |
| **ToF sensor** | ST VL53L5CX 8×8 zone, wrist-mounted, mm resolution |
| **IMU** | ST ISM330DHCX 6-axis on end-effector, contact detection at RMS > 3.5 dps |
| **Camera** | Pi Camera 3, overhead fixed mount |
| **Objects** | 1×1×1 inch styrofoam cubes — red, blue, green |
| **Workspace** | 60×60 cm white paper; arm base 5 cm from near edge; usable zone ≈ 440×160 mm trapezoid |

---

## What Is Already Done

- **YOLOv8-nano** fine-tuned on synthetic + real cube images: **mAP50 = 0.995** across three classes (`red_cube`, `blue_cube`, `green_cube`). Checkpoint at `checkpoints/yolov8n_vla/weights/best.pt`.
- **Full inference pipeline** (8 modules, 356 tests passing): inverse kinematics, 3-stage safety filter, skill FSM, language encoder, pose estimator, YOLO detector, serial comms, 8 Hz main loop.
- **Dataset pipeline** (4 files): HDF5 demo reader, rule-based skill segmenter, 4-way augmentation, PyTorch `VLADataset` — all tested end-to-end on synthetic data.
- **Live dashboard**: 5-panel PyQt6 GUI (joint angles, ToF heatmap, skill state, YOLO detections, telemetry strip) running at 10 Hz on synthetic data.

---

## Repository Structure

```
vla-robotic-arm/
├── README.md                          # this file
├── VLA_Training_README.md             # Colab training guide (cell-by-cell)
├── requirements.txt                   # Python dependencies
├── yolov8n.pt                         # YOLOv8-nano base weights
├── checkpoints/
│   └── yolov8n_vla/weights/
│       └── best.pt                    # fine-tuned cube detector
├── demos/                             # (empty) HDF5 demonstrations go here
├── dataset/
│   ├── hdf5_reader.py                 # load/inspect HDF5 demo files
│   ├── skill_segmenter.py             # rule-based REACH/GRASP/LIFT/PLACE labeller
│   ├── augmentation.py                # 4-way augmentation + build_training_set
│   └── vla_dataset.py                 # torch.utils.data.Dataset wrapping pipeline
└── rpi5_inference/
    ├── main.py                        # 8 Hz VLA inference entry point
    ├── config/
    │   ├── arm_config.yaml            # DH params, joint limits, workspace bounds
    │   └── model_config.yaml          # model checkpoint paths
    ├── calibration/                   # sensor calibration YAMLs (see below)
    ├── comms/
    │   └── teensy_serial.py           # USB serial link — 250-byte telemetry, 20-byte cmd
    ├── dashboard/
    │   └── gui.py                     # PyQt6 live 5-panel GUI
    ├── evaluation/
    │   └── run_eval.py                # task success-rate evaluator
    ├── language/
    │   └── language_encoder.py        # flan-t5-small, 512-dim L2-normalised embeddings
    ├── perception/
    │   ├── camera_manager.py          # Pi Camera 3 capture thread
    │   ├── pose_estimation.py         # homography + ToF depth → XYZ
    │   └── yolo_detector.py           # YOLOv8-nano wrapper + language-conditioned match
    ├── planning/
    │   ├── ik_solver.py               # closed-form 3-DOF inverse kinematics
    │   └── safety_filter.py           # 3-stage joint command validator
    └── vla/
        ├── skill_predictor.py         # 4-state FSM: REACH → GRASP → LIFT → PLACE
        ├── action_generator.py        # skill + pose → joint targets
        └── vla_policy.py              # full VLA policy forward pass
```

---

## What Ryan Needs To Deliver

### Serial Protocol

The Teensy firmware must produce and consume exactly these packet layouts over
`/dev/ttyACM0` at **2 Mbaud, 8N1**.  Checksum = XOR of all bytes excluding the
final checksum byte.

**Telemetry — Teensy → RPi5 at 50 Hz (250 bytes)**

```
Field            dtype       bytes   notes
────────────────────────────────────────────────────────────
magic            <u2            2    0xABCD
seq              <u4            4
timestamp_ms     <u4            4
servo_pos_raw    <u2 × 5       10    raw Dynamixel ticks
servo_vel        <i2 × 5       10    velocity (raw units)
servo_load       <i2 × 5       10    load (raw units, max ≈ ±1023)
imu_accel        <f4 × 3       12    m s⁻²
imu_gyro         <f4 × 3       12    rad s⁻¹
imu_gyro_rms     <f4            4    RMS angular rate, dps
contact_flag     u1             1    set when imu_gyro_rms > 3.5 dps
gripper_pos      <f4            4    jaw gap, mm
tof_grid         <u2 × 8×8   128    VL53L5CX distances, mm
adc_supply_mv    <u2            2    supply voltage, mV
temp_c           <i2            2    MCU temp × 100, °C
state_flags      u1             1
error_flags      u1             1
servo_temp       i1 × 5         5    °C per servo
reserved         u1 × 37       37
checksum         u1             1
────────────────────────────────────────────────────────────
TOTAL                         250 bytes
```

**Command — RPi5 → Teensy at 8 Hz (20 bytes)**

```
Field              dtype       bytes   notes
────────────────────────────────────────────────────────────
magic              u1             1    0xAA
seq                u1             1    wraps at 255
cmd_type           u1             1
joint_cmd_deg10    <i2 × 4        8    angle × 10 (123 = 12.3°)
gripper_cmd        u1             1    0 = closed, 100 = fully open
skill_state        u1             1    matches Skill IntEnum
flags              u1             1
reserved           u1 × 5         5
checksum           u1             1
────────────────────────────────────────────────────────────
TOTAL                            20 bytes
```

### Calibration Files

Place all six files in `rpi5_inference/calibration/`.

**`camera_intrinsics.yaml`** — Pi Camera 3 pinhole model
```yaml
# Obtained with OpenCV checkerboard calibration (≥20 images, 9×6 board)
image_width:  1920
image_height: 1080
camera_matrix:      # 3×3 K matrix
  rows: 3
  cols: 3
  data: [fx, 0, cx,
          0, fy, cy,
          0,  0,  1]
distortion_model: plumb_bob
distortion_coefficients:   # [k1, k2, p1, p2, k3]
  rows: 1
  cols: 5
  data: [k1, k2, p1, p2, k3]
```

**`homography.yaml`** — image pixel → workspace XY (metres)
```yaml
# H maps homogeneous pixel [u, v, 1] to workspace [X, Y, 1] in metres.
# Computed from ≥4 known-position calibration points on the workspace sheet.
homography:
  rows: 3
  cols: 3
  data: [h00, h01, h02,
         h10, h11, h12,
         h20, h21, h22]
```

**`servo_offsets.yaml`** — per-servo angle trim (degrees)
```yaml
# Add these offsets to joint_cmd_deg before encoding to ticks.
# Measured by commanding 0° and comparing observed arm pose to CAD.
offsets_deg:
  servo_1_j0:  0.0
  servo_2_j1a: 0.0
  servo_3_j1b: 0.0
  servo_4_j2:  0.0
  servo_5_j3:  0.0
```

**`workspace_corners.yaml`** — 4-corner boundary in workspace metres
```yaml
# Near-left, near-right, far-right, far-left (clockwise from operator view).
# Used to clip IK targets to valid reachable zone.
corners_m:
  - [x_near_left,  y_near_left]
  - [x_near_right, y_near_right]
  - [x_far_right,  y_far_right]
  - [x_far_left,   y_far_left]
```

**`tof_extrinsics.yaml`** — VL53L5CX pose on the wrist
```yaml
# Translation from end-effector origin to ToF sensor centre, in metres.
# Rotation as roll-pitch-yaw (degrees). Pointing down = pitch: -90.
translation_m: [tx, ty, tz]
rotation_rpy_deg: [roll, pitch, yaw]
```

**`imu_extrinsics.yaml`** — ISM330DHCX mount orientation
```yaml
# Rotation matrix aligning IMU body frame to end-effector frame.
# Identity if IMU X/Y/Z axes are already aligned with the arm's EE frame.
rotation:
  rows: 3
  cols: 3
  data: [1, 0, 0,
         0, 1, 0,
         0, 0, 1]
```

### Demo Dataset

Place **30 HDF5 demonstration files** in `demos/` following this convention:

```
demos/
  demo_001_pick_red_cube.h5
  demo_002_pick_blue_cube.h5
  demo_003_pick_green_cube.h5
  demo_004_stack_red_on_blue.h5
  ...
```

**HDF5 layout per file**

| Path | dtype | shape | notes |
|---|---|---|---|
| `/telemetry` | `uint8` | `(T, 250)` | raw `TELEMETRY_DTYPE` bytes, 50 Hz |
| `/rgb_frames` | `uint8` | `(F, H, W, 3)` | gzip-compressed overhead RGB |
| `/frame_ts` | `uint64` | `(F,)` | frame timestamps in microseconds |

**Required HDF5 attributes** (set on the root group):

| Attribute | type | example |
|---|---|---|
| `instruction` | str | `"pick up the red cube"` |
| `task_type` | str | `"pick"` / `"stack"` / `"sort"` |

**Minimum requirements per demo**

- **350 telemetry samples** (7 seconds at 50 Hz)
- All four skill phases present: REACH → GRASP → LIFT → PLACE
- `contact_flag` transitions from 0 to 1 during GRASP
- `imu_gyro_rms` > 3.5 dps at least once during GRASP impact
- `tof_grid` decreases toward target during REACH

---

## Setup Instructions

### RPi5 Setup

```bash
# System dependencies
sudo apt update && sudo apt install -y python3-pip libhdf5-dev python3-opencv

# Allow user access to Teensy serial port (log out and back in after)
sudo usermod -aG dialout $USER

# Enable Pi Camera 3 (legacy stack off, libcamera on)
sudo raspi-config   # → Interface Options → Camera → Enable

# Python environment
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# Smoke-test all modules (no hardware needed)
python dataset/hdf5_reader.py        # 28 tests
python dataset/skill_segmenter.py    # 42 tests
python dataset/augmentation.py       # 100 tests
python dataset/vla_dataset.py        # end-to-end dataset test
python -m rpi5_inference.main --dry-run
```

### Running

```bash
# Dry run — no Teensy or camera required
python -m rpi5_inference.main --dry-run

# Live inference
python -m rpi5_inference.main \
    --port /dev/ttyACM0 \
    --instruction "pick the red cube"

# Live dashboard (synthetic data, no hardware)
python -m rpi5_inference.dashboard.gui
```

### Training (after demos collected)

See **[VLA_Training_README.md](VLA_Training_README.md)** for the full
Colab notebook walkthrough.  Short version:

```python
from dataset.hdf5_reader     import load_all_demos
from dataset.skill_segmenter import segment_demo
from dataset.augmentation    import build_training_set
from dataset.vla_dataset     import VLADataset

demos      = load_all_demos("demos/")
timesteps  = [t for d in demos for t in segment_demo(d)]
train_set  = build_training_set(timesteps, augmentation_factor=4)
dataset    = VLADataset(train_set, language_encoder)
```

---

## Calibration Workflow

Run this once when the Pi Camera 3 arrives and the arm is mounted.

1. **Print** a 9×6 checkerboard (30 mm squares) and place it flat in the workspace.
2. **Capture ≥ 20 images** from the fixed overhead camera using `libcamera-still`.
3. **Run OpenCV calibration** to get the K matrix and distortion coefficients → `camera_intrinsics.yaml`.
4. **Place 4 calibration markers** at known XY positions on the workspace sheet (e.g., the four corners of a 300×200 mm rectangle).
5. **Record pixel coordinates** of each marker in the camera image.
6. **Compute homography** (`cv2.findHomography`) from the 4 pixel↔workspace pairs → `homography.yaml`.
7. **Zero-offset the servos**: command all joints to 0° and physically measure any visual offset → `servo_offsets.yaml`.
8. **Measure ToF mount**: with calipers, measure the translation from wrist flange to VL53L5CX face centre → `tof_extrinsics.yaml`.
9. **Record IMU orientation**: note which IMU axis points along the end-effector Z-axis → `imu_extrinsics.yaml`.
10. **Verify** by running `python -m rpi5_inference.main --dry-run` — pose estimation unit test will confirm homography is loaded.

---

## Evaluation Targets

| Task | Metric | Target |
|---|---|---|
| Task 1 — pick & place | Success rate | ≥ 85 % |
| Task 2 — stacking | Success rate | ≥ 75 % |
| Task 3 — sorting (3 cubes) | Success rate | ≥ 80 % |
| Inference latency | Per-step wall time | ≤ 125 ms (8 Hz budget) |
| Skill segmentation | Macro F1 | ≥ 75 % |
