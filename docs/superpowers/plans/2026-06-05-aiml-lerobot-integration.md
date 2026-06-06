# RoArm M2-S × LeRobot ACT — AI/ML Integration Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Integrate the RoArm M2-S (5-motor STS3215 arm) into HuggingFace LeRobot, train an ACT policy from 50-80 teleoperation demos, and deploy it on Raspberry Pi 5 for autonomous pick-and-place with an eye-in-hand camera.

**Architecture:** You write the custom robot config (Python, LeRobot source), record script, training pipeline, and rollout script. Ryan (hardware) calibrates the arm, collects demos, and runs the final checkpoint on RPi5. You work on a GPU machine for training; everything else runs on RPi5.

**Tech Stack:** Python 3.10+, HuggingFace LeRobot (ACT policy), FeetechMotorsBus (STS3215 servos), PyTorch + CUDA (training), Raspberry Pi 5 (inference).

---

## 0. Project Context

### The physical arm
The robot is a **Waveshare RoArm M2-S** — a 4-DOF arm with 5 STS3215 (Feetech) smart servos connected via a Waveshare USB servo adapter at `/dev/ttyACM0` on the RPi5. All servo commands flow directly from RPi5 → USB adapter → servo bus. There is no intermediate microcontroller in this path.

**Motor map (exact IDs, non-negotiable):**

| Name | Bus ID | Type | Norm Mode | Notes |
|---|---|---|---|---|
| `shoulder_pan` | 1 | sts3215 | DEGREES | Base yaw rotation |
| `shoulder_lift` | 2 | sts3215 | DEGREES | Shoulder pitch (primary) |
| `shoulder_lift_b` | 3 | sts3215 | DEGREES | Shoulder pitch (coupled) — always mirrors ID 2 |
| `elbow_flex` | 4 | sts3215 | DEGREES | Elbow / wrist combined |
| `gripper` | 5 | sts3215 | RANGE_0_100 | 0 = open, 100 = closed |

**Coupled shoulder:** ID 2 and ID 3 are mechanically linked — two servos driving the same shoulder joint for torque. Every command to `shoulder_lift` must be mirrored to `shoulder_lift_b`. The policy trains with a **5-dim action space** that includes `shoulder_lift_b`. In `send_action()`, the policy's `shoulder_lift_b` output is always overwritten with `shoulder_lift`'s value to enforce the coupling. The policy learns from demos that both values are identical — this is fine in practice.

**Camera:** USB camera mounted on the wrist (eye-in-hand), OpenCV index 0, 640×480 @ 30fps. The policy observes from this camera.

### LeRobot you already cloned
```
~/lerobot/   ← HuggingFace LeRobot repo already on the machine
```

Install path: `~/lerobot/src/lerobot/`. All custom files go inside this tree so they're importable as part of the package.

### What Ryan delivers to you

| Handoff | When | What |
|---|---|---|
| **H1** | Day 1 (after writing robot config) | Confirms motor IDs match, confirms camera index |
| **H2** | Day 3–4 (after collecting demos) | HuggingFace dataset repo ID, e.g. `ryanm/roarm_pickplace_v1` |
| **H3** | After training | You send checkpoint path; Ryan runs `lerobot-eval` on RPi5 |

---

## 1. File Map

Files you **create**:

```
~/lerobot/src/lerobot/robots/roarm_m2s/
├── __init__.py                     ← exports RoArmM2SFollower + RoArmM2SFollowerConfig
├── config_roarm_m2s.py             ← dataclass config, registers "roarm_m2s_follower" type
└── roarm_m2s_follower.py           ← robot class (subclasses SOFollower)

~/vla_rob/scripts/
├── record_roarm.py                 ← demo recording script (run on RPi5)
└── rollout_roarm.py                ← inference/deployment script (run on RPi5)
```

Files you **modify**:

```
~/lerobot/src/lerobot/robots/utils.py   ← add one elif branch to make_robot_from_config()
```

---

## Task 1: Environment Setup

**Files:** none (setup only)

- [ ] **Step 1: Install LeRobot on GPU machine with ACT + Feetech extras**

```bash
cd ~/lerobot
pip install -e ".[act, feetech]"
```

Expected output ends with: `Successfully installed lerobot-...`

- [ ] **Step 2: Verify ACT policy imports**

```bash
python -c "from lerobot.policies.act import ACTPolicy, ACTConfig; print('ACT OK')"
```

Expected: `ACT OK`

- [ ] **Step 3: Verify GPU is available**

```bash
python -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"
```

Expected: `True  NVIDIA ...`

If False: check CUDA driver, use `nvidia-smi` to confirm GPU is detected.

- [ ] **Step 4: On RPi5 — install LeRobot (CPU only, no training)**

SSH into RPi5 and run:
```bash
cd ~/lerobot
pip install -e ".[feetech]"
```

No GPU extras needed on RPi5 — it only runs inference.

- [ ] **Step 5: Commit (nothing to commit yet — setup only)**

```bash
cd ~/vla_rob
git status   # should be clean
```

---

## Task 2: Custom Robot Config

**Files:**
- Create: `~/lerobot/src/lerobot/robots/roarm_m2s/__init__.py`
- Create: `~/lerobot/src/lerobot/robots/roarm_m2s/config_roarm_m2s.py`
- Create: `~/lerobot/src/lerobot/robots/roarm_m2s/roarm_m2s_follower.py`
- Modify: `~/lerobot/src/lerobot/robots/utils.py`

- [ ] **Step 1: Write the failing import test**

Create `~/vla_rob/scripts/test_roarm_config.py`:

```python
"""Run this BEFORE implementing to confirm it fails, then again after to confirm it passes."""
from lerobot.robots.roarm_m2s import RoArmM2SFollower, RoArmM2SFollowerConfig

config = RoArmM2SFollowerConfig(port="/dev/ttyACM0", id="roarm_test")
robot = RoArmM2SFollower(config)

assert robot.name == "roarm_m2s_follower"
assert list(robot.bus.motors.keys()) == [
    "shoulder_pan", "shoulder_lift", "shoulder_lift_b", "elbow_flex", "gripper"
]
assert robot.bus.motors["shoulder_pan"].id == 1
assert robot.bus.motors["shoulder_lift"].id == 2
assert robot.bus.motors["shoulder_lift_b"].id == 3
assert robot.bus.motors["elbow_flex"].id == 4
assert robot.bus.motors["gripper"].id == 5
print("PASS: robot config correct")
```

- [ ] **Step 2: Run test to verify it fails**

```bash
cd ~/vla_rob
python scripts/test_roarm_config.py
```

Expected: `ModuleNotFoundError: No module named 'lerobot.robots.roarm_m2s'`

- [ ] **Step 3: Create config_roarm_m2s.py**

```python
# ~/lerobot/src/lerobot/robots/roarm_m2s/config_roarm_m2s.py
from dataclasses import dataclass

from ..config import RobotConfig
from ..so_follower.config_so_follower import SOFollowerConfig


@RobotConfig.register_subclass("roarm_m2s_follower")
@dataclass
class RoArmM2SFollowerConfig(RobotConfig, SOFollowerConfig):
    """Configuration for the Waveshare RoArm M2-S (5-motor STS3215 arm)."""
    pass
```

- [ ] **Step 4: Create roarm_m2s_follower.py**

```python
# ~/lerobot/src/lerobot/robots/roarm_m2s/roarm_m2s_follower.py
import logging
from functools import cached_property

from lerobot.cameras import make_cameras_from_configs
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.types import RobotAction

from ..robot import Robot
from ..so_follower.so_follower import SOFollower
from ..utils import ensure_safe_goal_position
from .config_roarm_m2s import RoArmM2SFollowerConfig

logger = logging.getLogger(__name__)


class RoArmM2SFollower(SOFollower):
    """
    Waveshare RoArm M2-S — 5-motor STS3215 arm with coupled shoulder.

    Differences from SO-100:
      - 5 motors instead of 6 (no wrist_roll)
      - shoulder_lift (ID2) and shoulder_lift_b (ID3) are mechanically coupled
        and always receive identical position commands
      - gripper is ID5 (not ID6)
    """

    config_class = RoArmM2SFollowerConfig
    name = "roarm_m2s_follower"

    def __init__(self, config: RoArmM2SFollowerConfig):
        # Call Robot.__init__ directly — SOFollower.__init__ would create a
        # 6-motor SO-100 bus which we immediately replace below.
        Robot.__init__(self, config)
        self.config = config
        norm_mode = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        self.bus = FeetechMotorsBus(
            port=self.config.port,
            motors={
                "shoulder_pan":    Motor(1, "sts3215", norm_mode),
                "shoulder_lift":   Motor(2, "sts3215", norm_mode),
                "shoulder_lift_b": Motor(3, "sts3215", norm_mode),
                "elbow_flex":      Motor(4, "sts3215", norm_mode),
                "gripper":         Motor(5, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=self.calibration,
        )
        self.cameras = make_cameras_from_configs(config.cameras)

    def calibrate(self) -> None:
        """Override SOFollower.calibrate() — removes hardcoded 'wrist_roll' logic."""
        if self.calibration:
            user_input = input(
                f"Press ENTER to use calibration file for '{self.id}', "
                "or type 'c' + ENTER to run new calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Writing calibration for {self.id} to motors")
                self.bus.write_calibration(self.calibration)
                return

        logger.info(f"Running calibration for {self}")
        self.bus.disable_torque()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input("Move RoArm M2-S to the MIDDLE of each joint's range, then press ENTER...")
        homing_offsets = self.bus.set_half_turn_homings()

        print(
            "Now move ALL joints sequentially through their FULL ranges of motion.\n"
            "Press ENTER when done..."
        )
        # All 5 motors have limited range — record all of them
        range_mins, range_maxes = self.bus.record_ranges_of_motion(list(self.bus.motors.keys()))

        self.calibration = {}
        for motor, m in self.bus.motors.items():
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print(f"Calibration saved to {self.calibration_fpath}")

    def send_action(self, action: RobotAction) -> RobotAction:
        """Send joint position command, mirroring shoulder_lift → shoulder_lift_b."""
        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        # Coupled shoulder: ID3 always mirrors ID2
        if "shoulder_lift" in goal_pos:
            goal_pos["shoulder_lift_b"] = goal_pos["shoulder_lift"]

        if self.config.max_relative_target is not None:
            present_pos = self.bus.sync_read("Present_Position")
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        self.bus.sync_write("Goal_Position", goal_pos)
        return {f"{motor}.pos": val for motor, val in goal_pos.items()}
```

- [ ] **Step 5: Create \_\_init\_\_.py**

```python
# ~/lerobot/src/lerobot/robots/roarm_m2s/__init__.py
from .config_roarm_m2s import RoArmM2SFollowerConfig
from .roarm_m2s_follower import RoArmM2SFollower

__all__ = ["RoArmM2SFollower", "RoArmM2SFollowerConfig"]
```

- [ ] **Step 6: Add elif branch to make_robot_from_config() in utils.py**

In `~/lerobot/src/lerobot/robots/utils.py`, find the block:
```python
    elif config.type == "so101_follower":
        from .so_follower import SO101Follower

        return SO101Follower(config)
```

Add IMMEDIATELY after it:
```python
    elif config.type == "roarm_m2s_follower":
        from .roarm_m2s import RoArmM2SFollower

        return RoArmM2SFollower(config)
```

- [ ] **Step 7: Run the import test — should now pass**

```bash
python ~/vla_rob/scripts/test_roarm_config.py
```

Expected: `PASS: robot config correct`

- [ ] **Step 8: Commit**

```bash
cd ~/lerobot
git add src/lerobot/robots/roarm_m2s/ src/lerobot/robots/utils.py
git commit -m "feat: add RoArm M2-S follower robot config (5-motor, coupled shoulder)"
```

---

## Task 3: Connection & Calibration Test

**Prerequisite:** Ryan has the arm connected to RPi5 at `/dev/ttyACM0`.
**Run these steps on RPi5.**

- [ ] **Step 1: Verify servos respond before connecting through LeRobot**

```bash
python -c "
from lerobot.robots.roarm_m2s import RoArmM2SFollower, RoArmM2SFollowerConfig
config = RoArmM2SFollowerConfig(port='/dev/ttyACM0', id='roarm_main')
robot = RoArmM2SFollower(config)
robot.bus.connect()
positions = robot.bus.sync_read('Present_Position')
print('Motor positions:', positions)
robot.bus.disconnect(disable_torque=False)
"
```

Expected: dict with 5 motor names and float degree values, e.g.:
```
Motor positions: {'shoulder_pan': 0.3, 'shoulder_lift': -12.1, 'shoulder_lift_b': -12.4, 'elbow_flex': 45.2, 'gripper': 15.0}
```

If you get a timeout or no response: check `ls /dev/ttyACM*` and confirm port.

- [ ] **Step 2: Run calibration**

```bash
lerobot-calibrate \
  --robot.type=roarm_m2s_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=roarm_main
```

Follow the on-screen prompts:
1. Move arm to middle of each joint's range → press Enter
2. Move all joints through full range → press Enter

Expected: `Calibration saved to ~/.cache/huggingface/lerobot/calibration/robots/roarm_m2s_follower/roarm_main.json`

- [ ] **Step 3: Verify calibration loaded on reconnect**

```bash
python -c "
from lerobot.robots.roarm_m2s import RoArmM2SFollower, RoArmM2SFollowerConfig
config = RoArmM2SFollowerConfig(port='/dev/ttyACM0', id='roarm_main')
robot = RoArmM2SFollower(config)
print('is_calibrated before connect:', robot.is_calibrated)
robot.connect(calibrate=False)
obs = robot.get_observation()
print('Observation keys:', list(obs.keys()))
print('shoulder_pan:', obs['shoulder_pan.pos'])
robot.disconnect()
"
```

Expected: `is_calibrated before connect: True` and observation keys including all 5 motor names.

- [ ] **Step 4: Test coupling — send a shoulder_lift command, confirm shoulder_lift_b mirrors it**

```bash
python -c "
import time
from lerobot.robots.roarm_m2s import RoArmM2SFollower, RoArmM2SFollowerConfig
config = RoArmM2SFollowerConfig(port='/dev/ttyACM0', id='roarm_main')
robot = RoArmM2SFollower(config)
robot.connect(calibrate=False)

# Command shoulder_lift to 10 degrees
action = {'shoulder_lift.pos': 10.0, 'shoulder_pan.pos': 0.0, 'elbow_flex.pos': 0.0, 'gripper.pos': 0.0}
robot.send_action(action)
time.sleep(1.0)

obs = robot.get_observation()
sl  = obs['shoulder_lift.pos']
slb = obs['shoulder_lift_b.pos']
print(f'shoulder_lift={sl:.1f}  shoulder_lift_b={slb:.1f}')
assert abs(sl - slb) < 5.0, f'COUPLING MISMATCH: {sl:.1f} vs {slb:.1f}'
print('PASS: coupling works')
robot.disconnect()
"
```

Expected: both values close to 10.0, `PASS: coupling works`

---

## Task 4: Demo Recording Script

**Files:**
- Create: `~/vla_rob/scripts/record_roarm.py`

Ryan has **one arm only — no leader arm**. Use LeRobot's built-in `GamepadTeleop`. Ryan will plug in a USB gamepad (any PlayStation/Xbox USB controller works on RPi5 out of the box).

- [ ] **Step 1: Write the recording script**

```python
#!/usr/bin/env python3
# ~/vla_rob/scripts/record_roarm.py
"""
Record teleoperation demonstrations for RoArm M2-S using a USB gamepad.

Requirements:
    pip install inputs   # gamepad library used by LeRobot's GamepadTeleop

Usage:
    python scripts/record_roarm.py \
        --follower_port /dev/ttyACM0 \
        --repo_id YOUR_HF_USERNAME/roarm_pickplace_v1 \
        --num_episodes 50

Gamepad controls (standard layout):
    Left stick  → shoulder_pan + shoulder_lift
    Right stick → elbow_flex
    R2 trigger  → gripper close
    L2 trigger  → gripper open

Recording controls:
    Right arrow (d-pad) → save episode, move to next
    Left arrow  (d-pad) → discard episode, re-record
    Start button        → stop recording early
"""

import argparse

from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.common.control_utils import init_keyboard_listener
from lerobot.datasets import LeRobotDataset, aggregate_pipeline_dataset_features, create_initial_features
from lerobot.processor import make_default_processors
from lerobot.robots.roarm_m2s import RoArmM2SFollower, RoArmM2SFollowerConfig
from lerobot.scripts.lerobot_record import record_loop
from lerobot.teleoperators.gamepad import GamepadTeleop, GamepadTeleopConfig
from lerobot.utils.feature_utils import combine_feature_dicts
from lerobot.utils.utils import log_say

FPS = 30
EPISODE_TIME_SEC = 30   # seconds per demo
RESET_TIME_SEC   = 10   # pause between demos
TASK_DESCRIPTION = "Pick the red cube and place it in the bin"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--follower_port", default="/dev/ttyACM0")
    parser.add_argument("--repo_id",       required=True, help="e.g. yourname/roarm_pickplace_v1")
    parser.add_argument("--num_episodes",  type=int, default=35)
    parser.add_argument("--camera_index",  type=int, default=0)
    args = parser.parse_args()

    camera_config = {
        "wrist": OpenCVCameraConfig(
            index_or_path=args.camera_index,
            width=640, height=480, fps=FPS,
        )
    }

    follower_config = RoArmM2SFollowerConfig(
        port=args.follower_port,
        id="roarm_main",
        cameras=camera_config,
        use_degrees=True,
    )
    teleop_config = GamepadTeleopConfig(use_gripper=True)

    follower = RoArmM2SFollower(follower_config)
    teleop   = GamepadTeleop(teleop_config)

    teleop_action_processor, _, robot_obs_processor = make_default_processors()

    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=FPS,
        features=combine_feature_dicts(
            aggregate_pipeline_dataset_features(
                pipeline=teleop_action_processor,
                initial_features=create_initial_features(action=follower.action_features),
                use_videos=True,
            ),
            aggregate_pipeline_dataset_features(
                pipeline=robot_obs_processor,
                initial_features=create_initial_features(observation=follower.observation_features),
                use_videos=True,
            ),
        ),
        robot_type=follower.name,
        use_videos=True,
        image_writer_threads=4,
    )

    teleop.connect()
    follower.connect()

    listener, events = init_keyboard_listener()

    try:
        episode_idx = 0
        while episode_idx < args.num_episodes and not events["stop_recording"]:
            log_say(f"Recording episode {episode_idx + 1} of {args.num_episodes}")

            record_loop(
                robot=follower,
                events=events,
                fps=FPS,
                teleop=teleop,
                dataset=dataset,
                control_time_s=EPISODE_TIME_SEC,
                single_task=TASK_DESCRIPTION,
                display_data=True,
            )

            if events["rerecord_episode"]:
                log_say("Re-recording episode")
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                continue

            dataset.save_episode()
            episode_idx += 1

            if episode_idx < args.num_episodes and not events["stop_recording"]:
                log_say("Reset the environment")
                record_loop(
                    robot=follower,
                    events=events,
                    fps=FPS,
                    teleop=teleop,
                    control_time_s=RESET_TIME_SEC,
                    single_task=TASK_DESCRIPTION,
                    display_data=False,
                )

    finally:
        log_say("Done recording")
        teleop.disconnect()
        follower.disconnect()
        listener.stop()
        dataset.finalize()
        dataset.push_to_hub()
        print(f"\nDataset pushed to HuggingFace: https://huggingface.co/datasets/{args.repo_id}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Install gamepad dependency and dry-run import check**

```bash
pip install inputs    # required by LeRobot's GamepadTeleop
cd ~/vla_rob
python -c "
from lerobot.teleoperators.gamepad import GamepadTeleop, GamepadTeleopConfig
from lerobot.robots.roarm_m2s import RoArmM2SFollower, RoArmM2SFollowerConfig
print('imports OK')
" 2>&1 | head -5
```

Expected: `imports OK`

- [ ] **Step 3: Commit**

```bash
cd ~/vla_rob
git add scripts/record_roarm.py
git commit -m "feat: add RoArm M2-S demo recording script"
```

---

## Task 5: ACT Training

**Prerequisite:** Ryan delivers dataset repo ID (Handoff H2).

Run all steps on the **GPU machine**.

- [ ] **Step 1: Confirm dataset exists on HuggingFace**

```bash
# Replace with the actual repo_id Ryan gives you
DATASET_ID="ryanm/roarm_pickplace_v1"
python -c "
from lerobot.datasets import LeRobotDatasetMetadata
meta = LeRobotDatasetMetadata('$DATASET_ID')
print('Episodes:', meta.total_episodes)
print('Frames:', meta.total_frames)
print('Features:', list(meta.features.keys())[:10])
"
```

Expected: `Episodes: 50`, `Frames: ~45000` (50 × 30s × 30fps), feature keys including `observation.state`, `action`, `observation.images.wrist`

- [ ] **Step 2: Inspect one episode to confirm it looks correct**

```bash
python -c "
from lerobot.datasets import LeRobotDataset
ds = LeRobotDataset('$DATASET_ID', episodes=[0])
frame = ds[0]
print('Frame keys:', list(frame.keys()))
print('Action shape:', frame['action'].shape)
print('State shape:', frame['observation.state'].shape)
print('Image shape:', frame['observation.images.wrist'].shape)
"
```

Expected:
- `action.shape` = `(5,)` — 5 motors
- `observation.state.shape` = `(5,)` — 5 motor positions
- `observation.images.wrist.shape` = `(3, 480, 640)` — CHW format

- [ ] **Step 3: Run ACT training**

```bash
lerobot-train \
  --dataset.repo_id=ryanm/roarm_pickplace_v1 \
  --policy.type=act \
  --policy.chunk_size=100 \
  --policy.n_action_steps=100 \
  --policy.input_normalization_modes.observation.state=mean_std \
  --policy.input_normalization_modes.observation.images.wrist=mean_std \
  --policy.output_normalization_modes.action=mean_std \
  --training.num_workers=4 \
  --training.batch_size=8 \
  --training.num_steps=80000 \
  --training.eval_freq=5000 \
  --training.save_freq=10000 \
  --output_dir=outputs/roarm_act_v1 \
  --wandb.enable=true \
  --wandb.project=roarm_act
```

Training time estimate: ~4 hours on RTX 3090, ~2 hours on A100.

- [ ] **Step 4: Monitor training**

Watch W&B for `l_action` (training loss) to fall below 0.1 within 30k steps. If it plateaus above 0.5:
- Confirm dataset has enough variation (re-check episode replays)
- Reduce `batch_size` to 4 if GPU OOM

- [ ] **Step 5: Evaluate on validation split**

After training completes, the best checkpoint is at `outputs/roarm_act_v1/checkpoints/last/pretrained_model/`.

```bash
python -c "
from lerobot.configs import PreTrainedConfig
cfg = PreTrainedConfig.from_pretrained('outputs/roarm_act_v1/checkpoints/last/pretrained_model')
print('Policy type:', cfg.type)
print('Action dim:', cfg.output_features)
"
```

Expected: `Policy type: act`, action dim shows 5 motors.

- [ ] **Step 6: Push checkpoint to HuggingFace**

```bash
lerobot-train \
  --policy.pretrained_path=outputs/roarm_act_v1/checkpoints/last/pretrained_model \
  --push_to_hub=true \
  --hub_id=YOUR_HF_USERNAME/roarm_act_v1
```

Or manually:
```bash
huggingface-cli upload YOUR_HF_USERNAME/roarm_act_v1 \
  outputs/roarm_act_v1/checkpoints/last/pretrained_model .
```

- [ ] **Step 7: Commit training config**

```bash
cd ~/vla_rob
# Document the exact training command for reproducibility
cat > scripts/train_act.sh << 'EOF'
#!/bin/bash
# Exact command used to train RoArm M2-S ACT policy
lerobot-train \
  --dataset.repo_id=ryanm/roarm_pickplace_v1 \
  --policy.type=act \
  --policy.chunk_size=100 \
  --policy.n_action_steps=100 \
  --training.num_steps=80000 \
  --output_dir=outputs/roarm_act_v1
EOF
chmod +x scripts/train_act.sh
git add scripts/train_act.sh
git commit -m "feat: add ACT training command for RoArm M2-S"
```

---

## Task 6: Rollout Deployment Script

**Files:**
- Create: `~/vla_rob/scripts/rollout_roarm.py`

> **⚠️ RPi5 inference speed:** RPi5 ARM CPU takes ~10–15 seconds per ACT inference pass. With 100-step action chunking, you need re-inference every ~3.3s but inference takes 10–15s — always waiting. **Do not run rollout on RPi5 CPU.** Instead, run rollout on the **GPU laptop** using `usbip` to expose `/dev/ttyACM0` from RPi5 as a local device. See Ryan's Task 7 for the `usbip` setup steps. The rollout script itself is identical either way.

**Run on GPU laptop (not RPi5).** No EE kinematics needed — policy operates directly in joint space.

- [ ] **Step 1: Write rollout script**

```python
#!/usr/bin/env python3
# ~/vla_rob/scripts/rollout_roarm.py
"""
Deploy a trained ACT policy on the RoArm M2-S for autonomous pick-and-place.

Usage (on RPi5):
    python scripts/rollout_roarm.py \
        --model_id YOUR_HF_USERNAME/roarm_act_v1 \
        --port /dev/ttyACM0 \
        --duration 60

Press Ctrl+C to stop cleanly.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # ~/vla_rob on sys.path
from firmware.tools.safety_listener import SafetyMonitor

from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.configs import PreTrainedConfig
from lerobot.robots.roarm_m2s import RoArmM2SFollower, RoArmM2SFollowerConfig
from lerobot.rollout import BaseStrategyConfig, RolloutConfig, build_rollout_context
from lerobot.rollout.inference import SyncInferenceConfig
from lerobot.rollout.strategies import BaseStrategy
from lerobot.utils.process import ProcessSignalHandler
from lerobot.utils.utils import init_logging

FPS = 30


def main():
    init_logging()

    parser = argparse.ArgumentParser()
    parser.add_argument("--model_id", required=True, help="HF model ID or local path to checkpoint")
    parser.add_argument("--port",     default="/dev/ttyACM0")
    parser.add_argument("--duration", type=int, default=60, help="Episode duration in seconds")
    parser.add_argument("--camera",   type=int, default=0, help="OpenCV camera index")
    args = parser.parse_args()

    camera_config = {
        "wrist": OpenCVCameraConfig(
            index_or_path=args.camera,
            width=640, height=480, fps=FPS,
        )
    }

    robot_config = RoArmM2SFollowerConfig(
        port=args.port,
        id="roarm_main",
        cameras=camera_config,
        use_degrees=True,
    )

    policy_config = PreTrainedConfig.from_pretrained(args.model_id)
    policy_config.pretrained_path = args.model_id

    cfg = RolloutConfig(
        robot=robot_config,
        policy=policy_config,
        strategy=BaseStrategyConfig(),
        inference=SyncInferenceConfig(),
        fps=FPS,
        duration=args.duration,
        task="Pick the red cube and place it in the bin",
    )

    signal_handler = ProcessSignalHandler(use_threads=True)

    monitor = SafetyMonitor(
        port="/dev/ttyUSB0",
        baud=2_000_000,
        on_estop=signal_handler.trigger_shutdown,
    )
    monitor.start()

    try:
        ctx = build_rollout_context(cfg, signal_handler.shutdown_event)
        strategy = BaseStrategy(cfg.strategy)
        try:
            strategy.setup(ctx)
            strategy.run(ctx)
        finally:
            strategy.teardown(ctx)
    finally:
        monitor.stop()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Dry-run import check on RPi5**

```bash
cd ~/vla_rob
python -c "import scripts.rollout_roarm; print('rollout imports OK')"
```

Expected: `rollout imports OK`

- [ ] **Step 3: Live test with trained checkpoint on RPi5**

```bash
python scripts/rollout_roarm.py \
  --model_id YOUR_HF_USERNAME/roarm_act_v1 \
  --duration 30
```

First run: arm should move toward the object it was trained to pick. If it moves erratically, check that camera index matches what was used during recording.

- [ ] **Step 4: Commit rollout script**

```bash
cd ~/vla_rob
git add scripts/rollout_roarm.py
git commit -m "feat: add ACT rollout deployment script for RPi5"
```

---

## Ryan ↔ Friend Handoffs Summary

### H1: After Task 2 (robot config)
Friend tells Ryan:
```
Robot config registered as "roarm_m2s_follower"
Motor names: shoulder_pan, shoulder_lift, shoulder_lift_b, elbow_flex, gripper
Camera key in dataset: wrist
Config class: RoArmM2SFollowerConfig(port="/dev/ttyACM0", id="roarm_main", ...)
```

Ryan confirms motor IDs and camera index before recording.

### H2: After Ryan collects demos
Ryan tells Friend:
```
Dataset pushed to HuggingFace: ryanm/roarm_pickplace_v1
50 episodes, task: "Pick the red cube and place it in the bin"
```

Friend starts Task 5 (training).

### H3: After training
Friend tells Ryan:
```
Checkpoint at: YOUR_HF_USERNAME/roarm_act_v1
Run: python scripts/rollout_roarm.py --model_id YOUR_HF_USERNAME/roarm_act_v1
```

Ryan runs rollout on RPi5.

---

## Appendix: Quick Reference Commands

```bash
# Calibrate (RPi5)
lerobot-calibrate --robot.type=roarm_m2s_follower --robot.port=/dev/ttyACM0 --robot.id=roarm_main

# Record demos (RPi5) — USB gamepad, no leader arm needed
python scripts/record_roarm.py \
  --follower_port /dev/ttyACM0 \
  --repo_id YOUR_HF_USERNAME/roarm_pickplace_v1 \
  --num_episodes 50

# Train (GPU machine)
bash scripts/train_act.sh

# Deploy (RPi5)
python scripts/rollout_roarm.py \
  --model_id YOUR_HF_USERNAME/roarm_act_v1 \
  --duration 60
```

Contact Ryan to confirm before implementing the recording script.
