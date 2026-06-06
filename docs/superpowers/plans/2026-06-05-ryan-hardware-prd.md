# RoArm M2-S — Ryan Hardware & Demo Collection PRD

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Mount a wrist camera, calibrate the arm via LeRobot, collect 50-80 teleoperated pick-and-place demonstrations, push the dataset to HuggingFace, and ship a simplified ESP32 safety monitor — all so the trained ACT checkpoint can be deployed for a clean hiring-level demo.

**Architecture:** Servo commands flow RPi5 → `/dev/ttyACM0` (SmartElex USB adapter) → STS3215 servos — directly via LeRobot's `FeetechMotorsBus`. The ESP32 on `/dev/ttyUSB0` is repurposed as a **safety-only monitor** (IMU contact detection + emergency stop). The broken 50Hz telemetry path is permanently abandoned — do not debug it.

**Tech Stack:** PlatformIO / C++ (ESP32 safety firmware), Python / LeRobot (calibration + recording + deploy), HuggingFace Hub (dataset + checkpoint storage).

---

## 0. Context Every New Chat Needs

### Hardware (verified working ✅)
- **Arm:** Waveshare RoArm M2-S — exact clone of SO-ARM100 but 5 motors, not 6
- **Servos:** 5× STS3215 (Feetech) smart servos on SmartElex USB adapter → `/dev/ttyACM0` on RPi5
- **ESP32:** DevKit V1 on `/dev/ttyUSB0` — IMU + ToF sensor hub (servos do NOT go through ESP32)
- **IMU:** ISM330DHCX on I2C (SDA=GPIO21, SCL=GPIO22, addr 0x6B) — verified WHO_AM_I ✅
- **ToF:** VL53L5CX on same I2C bus — verified ~175mm distance ✅
- **RPi5:** 8GB, runs Python inference, connected to both `/dev/ttyACM0` (servos) and `/dev/ttyUSB0` (ESP32)

### Motor map (IDs 1–5, non-negotiable)
| Motor name | Bus ID | Notes |
|---|---|---|
| `shoulder_pan` | 1 | Base yaw |
| `shoulder_lift` | 2 | Shoulder (primary) |
| `shoulder_lift_b` | 3 | Shoulder (coupled — always mirrors ID 2) |
| `elbow_flex` | 4 | Elbow |
| `gripper` | 5 | 0=open, 100=closed |

### Architecture decision: LeRobot replaces the custom VLA stack
The original `rpi5_inference/` custom VLA pipeline is **replaced** by HuggingFace LeRobot + ACT. Reasons:
1. LeRobot's `FeetechMotorsBus` already works with STS3215 on `/dev/ttyACM0`
2. ACT trains well on 50-80 demos — no custom policy code needed
3. The 50Hz ESP32 telemetry stream was broken (bad checksum, 64-byte repeating pattern) and the root cause was not diagnosed — with LeRobot we don't need it

### Division of labour
- **You (Ryan):** camera mount, calibration, demo collection, dataset push, ESP32 safety firmware
- **Friend (AI/ML):** writes LeRobot robot config, recording script, trains ACT, writes rollout script
- **Friend's plan:** `docs/superpowers/plans/2026-06-05-aiml-lerobot-integration.md`

### Critical path
```
You send motor spec → Friend writes robot config (Task 2)
                    ↓
You calibrate arm (Task 4) ← needs friend's Task 2 done
                    ↓
You collect 50-80 demos (Task 5) ← needs friend's Task 4 (record script) done
                    ↓
You push dataset → Friend trains ACT → Friend sends checkpoint
                    ↓
You deploy (Task 7)
```

### Teleoperation method — USB gamepad (decided)
No second arm needed. Use a **USB gamepad** (any PlayStation/Xbox USB controller, ~$15–25). LeRobot has a built-in `GamepadTeleop` — friend's record script already uses it. Buy a gamepad now so it arrives before you're ready to record. Plug into RPi5 USB, it works as a HID device out of the box with no drivers.

---

## 1. File Map

Files you **create**:
```
firmware/src/safety_monitor/
├── safety_main.cpp        ← stripped main.cpp, only IMU + contact oracle + safety packet
└── safety_comms.h         ← new 16-byte SafetyStatus_t struct

firmware/tools/
└── safety_listener.py     ← RPi5: reads safety packets, triggers estop callback
```

Files you **do not touch**:
```
firmware/src/ism330dhcx_driver.*   ← reused as-is (IMU driver works)
firmware/src/contact_oracle.*      ← reused as-is (already verified)
firmware/include/config.h          ← reused as-is
rpi5_inference/                    ← abandoned, do not modify
```

---

## Task 1: Send Motor Spec to Friend

**No hardware needed. Do this immediately.**

- [ ] **Step 1: Send this exact message to your friend**

```
Hey — here's the motor spec for our arm.
Add this to your roarm_m2s_follower.py config:

Motor map:
  shoulder_pan    → ID 1 (DEGREES norm)
  shoulder_lift   → ID 2 (DEGREES norm)
  shoulder_lift_b → ID 3 (DEGREES norm, always mirrors ID 2 — coupling logic in send_action)
  elbow_flex      → ID 4 (DEGREES norm)
  gripper         → ID 5 (RANGE_0_100 norm)

USB servo adapter port:  /dev/ttyACM0
Servo baud rate:         1,000,000 (1 Mbaud)
Camera:                  USB, OpenCV index 0, 640×480 @ 30fps, key name "wrist"
Teleop method:           USB gamepad (GamepadTeleop, no leader arm)
```

Fill in the teleop method before sending. This unblocks friend's Task 2.

---

## Task 2: Mount Camera on Wrist

**Hardware task — no software.**

- [ ] **Step 1: Choose a camera**

Use one of:
- **Pi Camera Module 3** (~4g, MIPI, needs adapter cable) — lightest option
- **Small USB webcam** (ELP, Logitech C270) — plug-and-play, ~30–40g, recommended

Rule: camera mass must be ≤ 50g. Anything heavier will noticeably increase load on J2/J3 servos.

- [ ] **Step 2: Mount position**

Mount the camera to the **wrist link — between J2 (elbow_flex) and J3 (gripper)**. It should point roughly forward along the gripper axis. The gripper should be visible in the bottom quarter of the frame when the arm is extended.

Do NOT mount past J3 (on the gripper jaw) — it will rotate with the gripper and ruin consistency.

- [ ] **Step 3: Route the cable**

Run the USB cable (or MIPI ribbon) along the arm body with zip ties or cable clips. Leave enough slack at each joint so the cable doesn't bind at the limits of motion. Test the full range of all joints — cable must not pull tight anywhere.

- [ ] **Step 4: Plug into RPi5 and verify device appears**

```bash
ls /dev/video*
```

Expected: `/dev/video0` (or `/dev/video1` if another camera is present)

```bash
v4l2-ctl --list-devices
```

Expected: your USB camera listed with `/dev/video0`

---

## Task 3: Verify Camera Stream

**Run on RPi5.**

- [ ] **Step 1: Install OpenCV if not present**

```bash
python -c "import cv2; print(cv2.__version__)"
```

If error: `pip install opencv-python-headless`

- [ ] **Step 2: Capture a test frame**

```bash
python -c "
import cv2, sys
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
ret, frame = cap.read()
cap.release()
if not ret:
    print('FAIL: no frame captured')
    sys.exit(1)
print(f'PASS: frame shape = {frame.shape}')  # should be (480, 640, 3)
cv2.imwrite('/tmp/wrist_test.jpg', frame)
print('Saved to /tmp/wrist_test.jpg — copy to your machine and inspect')
"
```

Expected: `PASS: frame shape = (480, 640, 3)`

To copy the test image to your dev machine:
```bash
scp pi@raspberrypi.local:/tmp/wrist_test.jpg ~/Desktop/
```

Confirm the frame shows the workspace from the wrist camera angle. The gripper should be visible at bottom.

- [ ] **Step 3: If camera is at index 1 instead of 0**

```bash
python -c "
import cv2
for idx in range(4):
    cap = cv2.VideoCapture(idx)
    ret, _ = cap.read()
    cap.release()
    print(f'index {idx}: {\"OK\" if ret else \"no device\"}')
"
```

Note the correct index and update the `--camera_index` arg when running the record script in Task 5.

---

## Task 4: Install LeRobot on RPi5 and Calibrate Arm

**Prerequisite: friend has completed their Task 2 (robot config written and pushed to `~/lerobot`).**

Confirm with friend: "Has `lerobot/src/lerobot/robots/roarm_m2s/` been created and pushed?"

- [ ] **Step 1: Install LeRobot on RPi5 (Feetech extras only — no GPU)**

```bash
cd ~/lerobot
git pull                          # get friend's robot config changes
pip install -e ".[feetech]"
```

Expected: `Successfully installed lerobot-...`

- [ ] **Step 2: Verify robot config imported**

```bash
python -c "
from lerobot.robots.roarm_m2s import RoArmM2SFollower, RoArmM2SFollowerConfig
config = RoArmM2SFollowerConfig(port='/dev/ttyACM0', id='roarm_main')
robot = RoArmM2SFollower(config)
print('Motors:', list(robot.bus.motors.keys()))
print('PASS')
"
```

Expected:
```
Motors: ['shoulder_pan', 'shoulder_lift', 'shoulder_lift_b', 'elbow_flex', 'gripper']
PASS
```

If error: friend's config changes haven't been pulled. Run `git pull` again.

- [ ] **Step 3: Check port permissions**

```bash
ls -la /dev/ttyACM0
sudo chmod 666 /dev/ttyACM0
```

Add yourself to dialout group to make permanent:
```bash
sudo usermod -aG dialout $USER
# Log out and back in for this to take effect
```

- [ ] **Step 4: Ping all 5 servos before calibrating**

```bash
python -c "
from lerobot.robots.roarm_m2s import RoArmM2SFollower, RoArmM2SFollowerConfig
config = RoArmM2SFollowerConfig(port='/dev/ttyACM0', id='roarm_main')
robot = RoArmM2SFollower(config)
robot.bus.connect()
positions = robot.bus.sync_read('Present_Position')
print('Positions:', positions)
robot.bus.disconnect(disable_torque=False)
"
```

Expected: all 5 motor names in the output dict with float degree values.
If a motor is missing: check its cable connection on the servo bus daisy-chain.

- [ ] **Step 5: Run calibration**

```bash
lerobot-calibrate \
  --robot.type=roarm_m2s_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=roarm_main
```

When prompted:
1. Move every joint to the **physical midpoint** of its range → press Enter
2. Move each joint slowly through its **full range** (one at a time is fine) → press Enter

Expected final line:
```
Calibration saved to ~/.cache/huggingface/lerobot/calibration/robots/roarm_m2s_follower/roarm_main.json
```

- [ ] **Step 6: Verify calibration loads correctly on reconnect**

```bash
python -c "
from lerobot.robots.roarm_m2s import RoArmM2SFollower, RoArmM2SFollowerConfig
from lerobot.cameras.opencv import OpenCVCameraConfig

config = RoArmM2SFollowerConfig(
    port='/dev/ttyACM0',
    id='roarm_main',
    cameras={'wrist': OpenCVCameraConfig(index_or_path=0, width=640, height=480, fps=30)},
    use_degrees=True,
)
robot = RoArmM2SFollower(config)
print('Calibrated:', robot.is_calibrated)
robot.connect(calibrate=False)
obs = robot.get_observation()
print('Observation keys:', list(obs.keys()))
print('shoulder_pan:', round(obs['shoulder_pan.pos'], 1), 'deg')
print('Image shape:', obs['wrist'].shape)  # should be (480, 640, 3)
robot.disconnect()
"
```

Expected:
```
Calibrated: True
Observation keys: ['shoulder_pan.pos', 'shoulder_lift.pos', 'shoulder_lift_b.pos', 'elbow_flex.pos', 'gripper.pos', 'wrist']
shoulder_pan: <some_value> deg
Image shape: (480, 640, 3)
```

---

## Task 5: Collect 30 Demonstrations

**Prerequisite: friend has completed their Task 4 (record script written and pushed).**

Confirm with friend: "Has `~/vla_rob/scripts/record_roarm.py` been pushed?"

### Before recording: set up your task scene

1. Clear a 40×40cm workspace in front of the arm
2. Place a marker or tape square as the **drop target** — fixed position, do not move it between demos
3. Use a brightly coloured object (red or yellow cube, ~3×3×3cm) as the pick target
4. Place the pick object within arm reach — roughly 15–25cm forward of the base

### Recording tips for good demo quality
- Each demo: place the cube in a **slightly different position** within a 10cm radius — this gives the policy spatial generalisation
- Keep movements **smooth and deliberate** — jerky motions cause the policy to learn noise
- Always **fully close the gripper** on the object before lifting
- Always **fully release** at the drop zone before returning home
- Aim for ~15–20 seconds per demo

- [ ] **Step 1: Log in to HuggingFace CLI (one time only)**

```bash
pip install huggingface_hub
huggingface-cli login
```

Paste your HuggingFace API token when prompted (get it from huggingface.co → Settings → Access Tokens → New token with write permissions).

- [ ] **Step 2: Set your HuggingFace username**

```bash
export HF_USER=$(huggingface-cli whoami | head -1)
echo "Will push dataset to: $HF_USER/roarm_pickplace_v1"
```

- [ ] **Step 3: Plug in USB gamepad and verify it's detected**

```bash
ls /dev/input/js*
# Expected: /dev/input/js0
python -c "import inputs; pads = inputs.devices.gamepads; print('Gamepads:', len(pads), pads)"
# Expected: Gamepads: 1 [<Gamepad ...>]
```

If `inputs` not installed: `pip install inputs`

- [ ] **Step 4: Run recording**

```bash
cd ~/vla_rob
python scripts/record_roarm.py \
  --follower_port /dev/ttyACM0 \
  --repo_id $HF_USER/roarm_pickplace_v1 \
  --num_episodes 50 \
  --camera_index 0
```

**Gamepad controls:**
- Left stick → shoulder_pan + shoulder_lift
- Right stick → elbow_flex
- R2 trigger → close gripper
- L2 trigger → open gripper

**Recording controls:**
- D-pad right → save episode, move to next
- D-pad left → discard this episode, re-record
- Start button → stop recording early

- [ ] **Step 5: Mid-session quality check after 5 demos**

Stop recording (Esc). Replay the first 3 demos to confirm they look good:

```bash
python -c "
from lerobot.datasets import LeRobotDataset
import numpy as np
ds = LeRobotDataset('$HF_USER/roarm_pickplace_v1', episodes=[0, 1, 2])
for i, frame in enumerate(ds):
    if i % 30 == 0:
        act = frame['action'].numpy()
        print(f'Frame {i:4d} | action: {np.round(act, 1)}')
    if i > 90:
        break
"
```

Expected: action values change smoothly across frames, no NaN, gripper column (last) transitions from ~0 to ~100 and back.

If actions look constant/frozen: leader arm may not be transmitting. Check its cable and port.

- [ ] **Step 6: Resume recording for remaining 30 demos**

```bash
python scripts/record_roarm.py \
  --follower_port /dev/ttyACM0 \
  --repo_id $HF_USER/roarm_pickplace_v1 \
  --num_episodes 50 \
  --camera_index 0
```

The script will auto-detect the existing dataset and resume from where you left off.

---

## Task 6: Push Dataset to HuggingFace

- [ ] **Step 1: Verify episode count before pushing**

```bash
python -c "
from lerobot.datasets import LeRobotDatasetMetadata
meta = LeRobotDatasetMetadata('$HF_USER/roarm_pickplace_v1', local_files_only=True)
print('Total episodes:', meta.total_episodes)
print('Total frames:', meta.total_frames)
print('FPS:', meta.fps)
"
```

Expected: `Total episodes: 50`, `Total frames: ~45000` (50 × ~30s × 30fps)

Only proceed if episode count is ≥ 50.

- [ ] **Step 2: Push to HuggingFace Hub**

```bash
python -c "
from lerobot.datasets import LeRobotDataset
ds = LeRobotDataset('$HF_USER/roarm_pickplace_v1', local_files_only=True)
ds.push_to_hub(private=False)
print('Dataset pushed successfully')
"
```

Expected: progress bars for video files, then `Dataset pushed successfully`.
Dataset URL: `https://huggingface.co/datasets/$HF_USER/roarm_pickplace_v1`

- [ ] **Step 3: Send dataset ID to friend (Handoff H2)**

Send friend this message:
```
Dataset ready: <YOUR_HF_USER>/roarm_pickplace_v1
50 episodes, task: "Pick the red cube and place it in the bin"
Camera key: wrist (640×480 @ 30fps)
Action dims: 5 (shoulder_pan, shoulder_lift, shoulder_lift_b, elbow_flex, gripper)
Start training.
```

---

## Task 7: Deploy Trained Checkpoint

**Prerequisite: friend has sent you a HuggingFace model ID, e.g. `friendname/roarm_act_v1`.**

> **⚠️ RPi5 inference speed:** RPi5 ARM CPU takes ~10–15 seconds per ACT inference pass. At 30fps you need one every 0.033s — even with 100-step action chunking (~3.3s of actions), you'll always be waiting on inference. **Solution:** Run `rollout_roarm.py` from the **GPU laptop**, and expose the RPi5's servo USB adapter over the network using `usbip`:
> ```bash
> # On RPi5 — export the USB device
> sudo modprobe usbip_host && sudo usbipd -D
> sudo usbip bind -b <bus-id-of-ttyACM0>   # find bus-id with: usbip list -l
>
> # On GPU laptop — attach it as a local device
> sudo modprobe vhci-hcd
> sudo usbip attach -r <rpi5-ip> -b <bus-id>
> # /dev/ttyACM0 now appears on the GPU laptop
> python scripts/rollout_roarm.py --model_id friendname/roarm_act_v1 --port /dev/ttyACM0
> ```
> Alternatively SSH into RPi5 and keep the servo port local, but serve inference over a TCP socket — ask friend to set this up if usbip is unavailable on your OS.

- [ ] **Step 1: Install rollout script dependency on RPi5**

```bash
cd ~/lerobot
pip install -e ".[feetech]"        # already done — just confirm
```

- [ ] **Step 2: Run rollout**

```bash
cd ~/vla_rob
python scripts/rollout_roarm.py \
  --model_id friendname/roarm_act_v1 \
  --port /dev/ttyACM0 \
  --duration 30 \
  --camera 0
```

First run: arm should start moving toward the pick object within 1–2 seconds of the episode starting.

- [ ] **Step 3: Evaluate — run 10 trials, note success rate**

Place the cube at a fresh position for each trial. Record pass/fail.

| Trial | Cube position | Result |
|---|---|---|
| 1 | center | |
| 2 | left 5cm | |
| ... | | |

Target: ≥ 7/10. If below 5/10 → tell friend to collect more demos and retrain (add 15 more demos at the failing positions).

- [ ] **Step 4: Record demo video for portfolio**

```bash
# On RPi5 — record the external view with your phone
# On the RPi5 terminal — show the live camera feed during rollout:
python -c "
import cv2
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    cv2.imshow('wrist', frame)
    if cv2.waitKey(1) == 27:
        break
cap.release()
"
```

Capture split-screen: phone recording the arm + screengrab of the wrist camera feed.

---

## Task 8: Simplify ESP32 to Safety Monitor (Parallel — independent of all other tasks)

**This runs in parallel with Tasks 1–7. Does not block demo collection.**

The existing firmware is over-engineered for the new architecture. Strip it down to one job: read IMU at 50Hz, detect contact events, send a 16-byte safety packet to RPi5.

**Files:**
- Create: `firmware/src/safety_monitor/safety_main.cpp`
- Create: `firmware/src/safety_monitor/safety_comms.h`
- Create: `firmware/tools/safety_listener.py`

### Sub-task 8A: New safety firmware

- [ ] **Step 1: Create safety_comms.h**

```cpp
// firmware/src/safety_monitor/safety_comms.h
#pragma once
#include <stdint.h>

#define SAFETY_MAGIC  0xBEEFCAFEUL

typedef struct __attribute__((packed)) {
    uint32_t magic;         // Always SAFETY_MAGIC — framing marker
    uint32_t timestamp_us;  // micros()
    uint8_t  contact_flag;  // 1 = contact oracle triggered (latched)
    float    contact_rms;   // Gyro RMS in deg/s (4 bytes)
    uint8_t  estop_active;  // 1 = this packet carries an ESTOP request
    uint16_t checksum;      // sum of bytes 0–13, truncated to uint16
} SafetyStatus_t;           // 16 bytes: 4+4+1+4+1+2

static_assert(sizeof(SafetyStatus_t) == 16, "SafetyStatus_t must be 16 bytes");
```

- [ ] **Step 2: Create safety_main.cpp**

```cpp
// firmware/src/safety_monitor/safety_main.cpp
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFi.h>
#include "safety_comms.h"
#include "../ism330dhcx_driver.h"
#include "../contact_oracle.h"
#include "config.h"

// ── Safety thresholds ─────────────────────────────────────────────────────────
// Contact oracle fires when 8-sample gyro RMS exceeds this (deg/s).
// Calibrate by watching contact_rms during normal motion vs. contact.
#define SAFETY_CONTACT_THRESHOLD  3.5f

static uint16_t compute_checksum(const uint8_t* data, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++) sum += data[i];
    return sum;
}

static void send_safety_packet(bool estop) {
    SafetyStatus_t pkt = {};
    pkt.magic        = SAFETY_MAGIC;
    pkt.timestamp_us = micros();
    pkt.contact_flag = contact_oracle_triggered() ? 1 : 0;
    pkt.contact_rms  = contact_oracle_rms();
    pkt.estop_active = estop ? 1 : 0;
    pkt.checksum     = compute_checksum(
        (const uint8_t*)&pkt,
        sizeof(SafetyStatus_t) - sizeof(pkt.checksum)
    );
    Serial.write((const uint8_t*)&pkt, sizeof(SafetyStatus_t));
}

void safety_task(void* pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);  // 50Hz

    while (true) {
        imu_fifo_read_batch();  // Reads IMU FIFO, internally calls contact_oracle_push()

        // Escalate to ESTOP if contact RMS is way above threshold (hard collision)
        bool estop = (contact_oracle_rms() > SAFETY_CONTACT_THRESHOLD * 3.0f);

        send_safety_packet(estop);

        vTaskDelayUntil(&lastWakeTime, period);
    }
}

void setup() {
    WiFi.mode(WIFI_OFF);
    btStop();

    Serial.begin(USB_BAUD);
    delay(500);
    Serial.println("--- SAFETY MONITOR FIRMWARE ---");

    imu_init();
    contact_oracle_init(SAFETY_CONTACT_THRESHOLD);

    Serial.println("IMU ready. Starting safety task at 50Hz.");

    xTaskCreatePinnedToCore(safety_task, "SafetyTask", 4096, NULL, 10, NULL, 1);
    vTaskDelete(NULL);
}

void loop() {}
```

- [ ] **Step 3: Update platformio.ini to point at new main file**

Open `firmware/platformio.ini` and add `build_src_filter`:

```ini
[env:esp32dev]
platform    = espressif32
board       = esp32dev
framework   = arduino
build_flags = -O2 -std=c++17 -DCORE_DEBUG_LEVEL=0
monitor_speed = 2000000
build_src_filter = +<*> -<main.cpp> +<safety_monitor/safety_main.cpp>
lib_deps =
    Wire
    SPI
    stm32duino/STM32duino VL53L5CX @ ^1.2.3
```

This keeps `main.cpp` in the tree (do not delete it) but builds `safety_main.cpp` instead.

- [ ] **Step 4: Flash and verify**

```bash
cd ~/vla_rob/firmware
pio run --target upload
pio device monitor
```

Expected serial output (one line per second showing 50Hz packets are flowing):
```
--- SAFETY MONITOR FIRMWARE ---
IMU ready. Starting safety task at 50Hz.
```

After those two lines, binary packets flow — the monitor will show garbage. That's correct.

- [ ] **Step 5: Commit firmware**

```bash
cd ~/vla_rob
git add firmware/src/safety_monitor/ firmware/platformio.ini
git commit -m "feat: replace full telemetry firmware with 50Hz IMU safety monitor"
```

### Sub-task 8B: RPi5 safety listener

- [ ] **Step 6: Write safety_listener.py**

```python
# firmware/tools/safety_listener.py
"""
Background safety monitor — reads 16-byte SafetyStatus_t packets from ESP32
at 50Hz and calls on_estop() if a hard contact event is detected.

Usage in your rollout script:
    from firmware.tools.safety_listener import SafetyMonitor
    monitor = SafetyMonitor(on_estop=lambda: robot.disconnect())
    monitor.start()
    # ... run policy ...
    monitor.stop()
"""
import struct
import threading
import logging
import serial

logger = logging.getLogger(__name__)

SAFETY_MAGIC      = 0xBEEFCAFE
PACKET_SIZE       = 16
MAGIC_BYTES       = SAFETY_MAGIC.to_bytes(4, 'little')
PACKET_FMT        = '<IIBfBH'    # magic(4), ts(4), contact_flag(1), rms(4), estop(1), checksum(2)
assert struct.calcsize(PACKET_FMT) == PACKET_SIZE


def _verify_checksum(raw: bytes) -> bool:
    computed = sum(raw[:-2]) & 0xFFFF
    received = int.from_bytes(raw[-2:], 'little')
    return computed == received


class SafetyMonitor:
    """Thread that continuously reads safety packets and fires on_estop() on hard contact."""

    def __init__(self, port: str = '/dev/ttyUSB0', baud: int = 2_000_000, on_estop=None):
        self.port     = port
        self.baud     = baud
        self.on_estop = on_estop
        self._stop    = threading.Event()
        self._thread  = threading.Thread(target=self._run, daemon=True)
        self.last_rms = 0.0
        self.contact  = False

    def start(self):
        self._thread.start()
        logger.info("SafetyMonitor started on %s", self.port)

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _find_packet(self, ser: serial.Serial):
        """Scan stream for magic bytes, read rest of packet, verify checksum."""
        tail = bytearray()
        while not self._stop.is_set():
            b = ser.read(1)
            if not b:
                return None
            tail += b
            if len(tail) < 4:
                continue
            if bytes(tail[-4:]) != MAGIC_BYTES:
                if len(tail) > 4:
                    tail = tail[-4:]
                continue
            rest = ser.read(PACKET_SIZE - 4)
            if len(rest) < PACKET_SIZE - 4:
                return None
            pkt = bytes(tail[-4:]) + rest
            if _verify_checksum(pkt):
                return pkt
            tail = bytearray(rest[-3:])
        return None

    def _run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1.0)
        except serial.SerialException as e:
            logger.warning("SafetyMonitor could not open %s: %s", self.port, e)
            return

        try:
            while not self._stop.is_set():
                raw = self._find_packet(ser)
                if raw is None:
                    continue
                _, _, contact_flag, rms, estop_active, _ = struct.unpack(PACKET_FMT, raw)
                self.last_rms = rms
                self.contact  = bool(contact_flag)
                if estop_active and self.on_estop:
                    logger.warning("SafetyMonitor: ESTOP received (contact_rms=%.2f)", rms)
                    self.on_estop()
        finally:
            ser.close()
```

- [ ] **Step 7: Test the listener against live ESP32**

```bash
python -c "
import time
import sys
sys.path.insert(0, '/home/m0mspagetthi/vla_rob')
from firmware.tools.safety_listener import SafetyMonitor

def on_estop():
    print('ESTOP triggered!')

monitor = SafetyMonitor(port='/dev/ttyUSB0', baud=2_000_000, on_estop=on_estop)
monitor.start()
for _ in range(10):
    time.sleep(0.5)
    print(f'contact={monitor.contact}  rms={monitor.last_rms:.3f} deg/s')
monitor.stop()
print('Monitor stopped cleanly.')
"
```

Expected: 10 lines with `contact=False  rms=<small_value>` while the arm is stationary. When you bump the arm: `rms` spikes.

- [ ] **Step 8: Commit listener**

```bash
cd ~/vla_rob
git add firmware/tools/safety_listener.py
git commit -m "feat: add RPi5 safety listener for ESP32 contact monitor"
```

---

## Dependency Summary

| Your task | Blocked waiting for |
|---|---|
| Task 1 (send motor spec) | Nothing — do immediately |
| Task 2 (camera mount) | Nothing — do immediately |
| Task 3 (verify camera) | Task 2 |
| Task 4 (calibrate) | Friend's Task 2 (robot config) complete |
| Task 5 (collect demos) | Task 3 + Task 4 + Friend's Task 4 (record script) |
| Task 6 (push dataset) | Task 5 |
| Task 7 (deploy) | Friend's Task 5 (training) complete |
| Task 8 (safety firmware) | Nothing — fully parallel |

## What NOT To Do

- **Do not debug the 50Hz ESP32 telemetry stream.** The `verify_telemetry.py` failure (64-byte repeating pattern, bad checksums) is a known issue. Root cause: likely TX buffer overflow or task timing. We abandoned this path — servo control now goes directly through `/dev/ttyACM0`.
- **Do not modify `rpi5_inference/`.** The custom VLA pipeline there is superseded by LeRobot ACT. Leave it as-is for historical reference.
- **Do not change motor IDs on the servos.** They are already configured at IDs 1–5. Running `lerobot-setup-motors` again would reset them and require reconfiguration.

---

## Quick Reference

```bash
# Check servos respond (RPi5)
python -c "
from lerobot.robots.roarm_m2s import RoArmM2SFollower, RoArmM2SFollowerConfig
r = RoArmM2SFollower(RoArmM2SFollowerConfig(port='/dev/ttyACM0', id='roarm_main'))
r.bus.connect(); print(r.bus.sync_read('Present_Position')); r.bus.disconnect(False)
"

# Calibrate (RPi5)
lerobot-calibrate --robot.type=roarm_m2s_follower --robot.port=/dev/ttyACM0 --robot.id=roarm_main

# Record demos (RPi5) — USB gamepad, replace $HF_USER with your HuggingFace username
python ~/vla_rob/scripts/record_roarm.py \
  --follower_port /dev/ttyACM0 \
  --repo_id $HF_USER/roarm_pickplace_v1 --num_episodes 50

# Deploy trained checkpoint (RPi5)
python ~/vla_rob/scripts/rollout_roarm.py \
  --model_id friendname/roarm_act_v1 --duration 30

# Flash safety firmware (dev machine → RPi5 via SSH or direct)
cd ~/vla_rob/firmware && pio run --target upload
```
