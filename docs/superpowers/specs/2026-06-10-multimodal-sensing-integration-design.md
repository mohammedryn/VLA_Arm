# Multimodal Sensing Integration (ToF + IMU + Language Conditioning) — Design Spec

**Date:** 2026-06-10
**Author:** Ryan (hardware/embedded), via design session
**Status:** Approved for implementation planning

## Supersedes / Extends

- **Supersedes** `docs/superpowers/plans/2026-06-05-ryan-hardware-prd.md` Task 8 (ESP32 safety monitor) and the
  already-committed `firmware/src/safety_monitor/safety_main.cpp`, `firmware/src/safety_monitor/safety_comms.h`,
  and `firmware/tools/safety_listener.py`. The ESP32 DevKit V1 is removed from the architecture entirely.
- **Extends** `docs/superpowers/plans/2026-06-05-aiml-lerobot-integration.md`. The servo control path
  (RPi5 → `/dev/ttyACM_servo` → `FeetechMotorsBus` → 5x STS3215), the wrist USB camera, `GamepadTeleop`,
  and the overall `LeRobotDataset.create()` / `record_loop()` structure from that plan are **unchanged**.
  This spec adds two new observation modalities and language-phrase variation to `scripts/record_roarm.py`,
  which has not yet been created — this spec defines its v1 design directly (no migration needed).
- Does **not** touch `rpi5_inference/`, motor IDs, or the abandoned 250-byte telemetry protocol — all still
  out of scope per the existing PRD.

---

## 1. Goals / Non-Goals

**Goals:**
- Add wrist-area depth sensing (VL53L5CX, 8x8 ToF grid) and end-effector inertial sensing (ISM330DHCX IMU)
  as policy observations from demo episode 1 — no re-collection later.
- Preserve the existing safety function (contact detection → e-stop) without a separate safety MCU.
- Add language-phrase variation (2 phrases) to `single_task` from episode 1, enabling a future
  language-conditioning ablation.
- Consolidate onto hardware already in hand (Teensy 4.1, RPi5, RPi Pico W spare) and match the project's
  stated hardware target (Teensy 4.1 / PlatformIO / 50Hz loop per `CLAUDE.md`).

**Non-Goals:**
- Training/policy-side ACT config changes (friend's scope) — this spec only defines the dataset schema
  those changes consume.
- Camera mounting / calibration (separate task in the v1 PRD, unaffected by this spec).
- Full language-conditioning validation (3+ phrases, 30-50 demos each) — deferred to v3 with a larger dataset.
- Repurposing the ESP32 or RPi Pico W — both become spares, no firmware written for them here.

---

## 2. Architecture

```
                         RPi 5 (record_roarm.py, 30Hz record loop)
        ┌──────────────────────┬──────────────────────┬──────────────────────────┐
        │ /dev/roarm_servo      │ USB wrist camera      │ /dev/roarm_teensy         │
        │ (FeetechMotorsBus,    │ (cv2 idx 0,           │ (Teensy 4.1 native USB,   │
        │  unchanged)           │  640x480@30fps)       │  SensorStatus_t @ 50Hz)   │
        ▼                       ▼                       ▼
   5x STS3215 servos       wrist RGB image         Teensy 4.1 sensor+safety co-proc
   (shoulder_pan,                                    @ 50Hz loop:
    shoulder_lift,                                    - ISM330DHCX (IMU, I2C, 0x6B)
    shoulder_lift_b,                                    → contact oracle + estop
    elbow_flex, gripper)                              - VL53L5CX (ToF, I2C, 0x52)
                                                         → 8x8 grid @ ~15Hz
                                                       - packs both into one
                                                         168-byte packet, streams
                                                         continuously over USB CDC
```

- **ESP32 DevKit V1**: removed from the system. **RPi Pico W**: unused spare.
- **Teensy 4.1** is powered via USB from the RPi5 (5V), separate from the 12V servo rail — consistent with
  the project's power-isolation constraint. The IMU and ToF sensors are powered from the Teensy's 3.3V rail.
- **Two USB-serial links from RPi5**: the Feetech servo adapter and the Teensy. Both can enumerate as
  `/dev/ttyACM*` in either order across reboots — Section 6 adds udev rules for stable names
  `/dev/roarm_servo` and `/dev/roarm_teensy`.
- Both the safety function (contact/estop) and the new sensing observations (`observation.tof`,
  `observation.imu`) come from the **same packet** — one source of truth, one timestamp domain.

---

## 3. Hardware / Wiring Changes

| Signal | Old (ESP32, GPIO) | New (Teensy 4.1, pin) |
|---|---|---|
| I2C SDA (IMU + ToF shared bus) | GPIO21 | Pin 18 (`Wire` default SDA) |
| I2C SCL (IMU + ToF shared bus) | GPIO22 | Pin 19 (`Wire` default SCL) |
| ToF `LPN` (power enable, HIGH=on) | GPIO27 | Pin 2 |
| ToF `INT` (data-ready, active LOW) | GPIO26 | Pin 3 |
| ToF / IMU 3.3V + GND | ESP32 3.3V rail | Teensy 4.1 3.3V rail |
| Host link | `/dev/ttyUSB0` (CP2102, 2 Mbaud) | `/dev/ttyACM*` (Teensy native USB CDC) |

I2C addresses (0x6B for IMU, 0x52 for ToF) and the IMU's internal config (208Hz ODR, ±2g/±250dps,
FIFO continuous mode batching accel+gyro) are **unchanged** — only the host MCU and its pin map change.
This is a 6-wire rewire (SDA, SCL, LPN, INT, 3V3, GND) from the ESP32 dev board to the Teensy 4.1.

**Bring-up note:** re-run the equivalent of `firmware/test_sketches/imu_whoami` and
`firmware/test_sketches/tof_distance` against the Teensy 4.1 + new pins before integrating, to confirm
the rewire and that `STM32duino VL53L5CX` builds under `framework = arduino, board = teensy41`. This is
a verification step for the implementation plan, not a design change.

---

## 4. Firmware: Teensy 4.1 Sensor + Safety Co-Processor

### 4.1 File map

**New:**
- `firmware/src/sensor_safety/sensor_main.cpp` — Teensy entry point, 50Hz loop
- `firmware/src/sensor_safety/sensor_comms.h` — `SensorStatus_t` packet definition
- `firmware/src/sensor_safety/sensor_config.h` — Teensy pin map + tunables

**Modified (ported from ESP32 → Teensy, logic unchanged):**
- `firmware/src/ism330dhcx_driver.cpp` — replace ESP32-specific `Wire.begin(sda, scl)` with `Wire.begin()`;
  `#include "sensor_config.h"` instead of `config.h`
- `firmware/src/tof_driver.cpp` — same `Wire.begin()` change; `#include "sensor_config.h"`
- `firmware/platformio.ini` — add `[env:teensy41]`, remove `[env:esp32dev]`

**Unchanged, reused as-is:**
- `firmware/src/contact_oracle.h` / `.cpp` — pure math, no hardware dependency
- `firmware/src/ism330dhcx_driver.h`, `firmware/src/tof_driver.h` — interfaces unchanged

**Removed:**
- `firmware/src/safety_monitor/` (entire directory — `safety_main.cpp`, `safety_comms.h`)
- `firmware/tools/safety_listener.py`

### 4.2 `sensor_comms.h`

```cpp
// firmware/src/sensor_safety/sensor_comms.h
#pragma once
#include <stdint.h>

#define SENSOR_PACKET_MAGIC  0xBEEFCAFEUL

typedef struct __attribute__((packed)) {
    uint32_t magic;          // Always SENSOR_PACKET_MAGIC — framing marker
    uint32_t timestamp_us;   // micros() since Teensy boot
    uint8_t  contact_flag;   // 1 = contact oracle triggered (latched)
    float    contact_rms;    // Gyro RMS in deg/s
    uint8_t  estop_active;   // 1 = hard collision, RPi5 must stop
    float    accel[3];       // ax, ay, az in m/s^2
    float    gyro[3];        // gx, gy, gz in deg/s
    uint16_t tof_mm[64];     // 8x8 grid, row-major, mm; 0xFFFF = invalid zone
    uint16_t checksum;       // sum of bytes 0..165, truncated to uint16
} SensorStatus_t;             // 168 bytes total

static_assert(sizeof(SensorStatus_t) == 168, "SensorStatus_t must be 168 bytes");
```

### 4.3 `sensor_config.h`

```cpp
// firmware/src/sensor_safety/sensor_config.h
#pragma once

// ── I2C bus (Teensy 4.1 default Wire: SDA=18, SCL=19) ───────────────────────
#define IMU_I2C_ADDR            0x6B
#define TOF_I2C_ADDR            0x52
#define TOF_LPN                 2      // power enable, HIGH=powered
#define TOF_INT                 3      // data-ready, active LOW
#define TOF_UPDATE_HZ           15

// ── Contact oracle ───────────────────────────────────────────────────────────
#define CONTACT_WINDOW          8       // samples, ~38ms at 208Hz
#define SAFETY_CONTACT_THRESHOLD 3.5f   // deg/s RMS, calibrated value carried over

// ── Control loop ─────────────────────────────────────────────────────────────
#define CONTROL_HZ              50
#define CONTROL_PERIOD_US       20000   // 20ms
```

### 4.4 `sensor_main.cpp`

No FreeRTOS — only one job runs on this MCU now, so a plain timed `loop()` is simpler and matches the
Teensy Arduino core idiom.

```cpp
// firmware/src/sensor_safety/sensor_main.cpp
#include <Arduino.h>
#include <string.h>
#include "sensor_comms.h"
#include "sensor_config.h"
#include "../ism330dhcx_driver.h"
#include "../tof_driver.h"
#include "../contact_oracle.h"

static uint16_t compute_checksum(const uint8_t* data, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++) sum += data[i];
    return sum;
}

static elapsedMicros loop_timer;

void setup() {
    Serial.begin(115200);  // baud ignored by Teensy native USB CDC; kept for pyserial API compat

    imu_init();
    tof_init();
    contact_oracle_init(SAFETY_CONTACT_THRESHOLD);

    loop_timer = 0;
}

void loop() {
    if (loop_timer < CONTROL_PERIOD_US) return;
    loop_timer -= CONTROL_PERIOD_US;

    imu_fifo_read_batch();   // updates latest IMU sample, feeds contact_oracle_push()
    tof_check_ready();       // refreshes latest ToF frame if a new one is ready (~15Hz)

    bool estop = (contact_oracle_rms() > SAFETY_CONTACT_THRESHOLD * 3.0f);

    ImuData  imu = imu_get_latest();
    ToFFrame tof = tof_get_latest();

    SensorStatus_t pkt = {};
    pkt.magic        = SENSOR_PACKET_MAGIC;
    pkt.timestamp_us = micros();
    pkt.contact_flag = contact_oracle_triggered() ? 1 : 0;
    pkt.contact_rms  = contact_oracle_rms();
    pkt.estop_active = estop ? 1 : 0;
    pkt.accel[0] = imu.ax; pkt.accel[1] = imu.ay; pkt.accel[2] = imu.az;
    pkt.gyro[0]  = imu.gx; pkt.gyro[1]  = imu.gy; pkt.gyro[2]  = imu.gz;
    memcpy(pkt.tof_mm, tof.distances_mm, sizeof(pkt.tof_mm));
    pkt.checksum = compute_checksum((const uint8_t*)&pkt, sizeof(SensorStatus_t) - sizeof(pkt.checksum));

    Serial.write((const uint8_t*)&pkt, sizeof(SensorStatus_t));
}
```

### 4.5 Driver changes (`ism330dhcx_driver.cpp`, `tof_driver.cpp`)

Both files currently call `Wire.begin(TOF_SDA, TOF_SCL)` — an ESP32-only overload that remaps I2C pins.
Teensy 4.1's `Wire` uses fixed pins (18/19). Change in both files:

```cpp
// before (ESP32):
Wire.begin(TOF_SDA, TOF_SCL);
Wire.setClock(400000);

// after (Teensy 4.1):
Wire.begin();
Wire.setClock(400000);
```

And change the include from `"config.h"` to `"sensor_config.h"` (only `IMU_I2C_ADDR`, `TOF_I2C_ADDR`,
`TOF_LPN`, `TOF_INT`, `TOF_UPDATE_HZ` are needed by these drivers — all present in the new header with
the same names/values). No other logic changes — IMU register config, FIFO batching, contact-oracle
math, and ToF zone validity logic are identical to the working ESP32 version.

### 4.6 `platformio.ini`

```ini
[env:teensy41]
platform      = teensy
board         = teensy41
framework     = arduino
build_flags   = -O2 -std=c++17
build_src_filter = +<*> -<main.cpp> -<safety_monitor/*> +<sensor_safety/sensor_main.cpp>
lib_deps =
    Wire
    stm32duino/STM32duino VL53L5CX @ ^1.2.3
```

The `[env:esp32dev]` section is removed. `firmware/src/safety_monitor/` is excluded from the build
(then deleted as part of implementation cleanup). `firmware/src/main.cpp` and the abandoned
250-byte-telemetry sources remain excluded and untouched, per the existing PRD.

---

## 5. RPi5: Sensor Listener

Replaces `firmware/tools/safety_listener.py` with a listener for the new 168-byte packet. Same
magic-byte-scan + checksum framing pattern as the original, extended to parse the new fields and expose
a "latest reading" snapshot for the recording loop to poll.

```python
# firmware/tools/sensor_listener.py
"""
Background reader for the Teensy 4.1 sensor+safety co-processor.
Reads 168-byte SensorStatus_t packets at 50Hz, exposes the latest IMU/ToF/contact
state, and calls on_estop() if a hard contact event is detected.

Usage:
    from firmware.tools.sensor_listener import SensorMonitor
    monitor = SensorMonitor(on_estop=lambda: robot.disconnect())
    monitor.start()
    obs = monitor.latest_observation()  # {"tof": (8,8) float32, "imu": (6,) float32}
    monitor.stop()
"""
import struct
import threading
import logging
import numpy as np
import serial

logger = logging.getLogger(__name__)

SENSOR_MAGIC = 0xBEEFCAFE
PACKET_SIZE  = 168
MAGIC_BYTES  = SENSOR_MAGIC.to_bytes(4, 'little')
# magic(4) ts(4) contact_flag(1) rms(4) estop(1) accel(3f) gyro(3f) tof(64H) checksum(2)
PACKET_FMT   = '<IIBfB3f3f64HH'
assert struct.calcsize(PACKET_FMT) == PACKET_SIZE

TOF_MAX_RANGE_M = 4.0  # value substituted for invalid (0xFFFF) zones


def _verify_checksum(raw: bytes) -> bool:
    computed = sum(raw[:-2]) & 0xFFFF
    received = int.from_bytes(raw[-2:], 'little')
    return computed == received


class SensorMonitor:
    """Thread that continuously reads sensor packets and fires on_estop() on hard contact."""

    def __init__(self, port: str = '/dev/roarm_teensy', baud: int = 115200, on_estop=None):
        self.port         = port
        self.baud         = baud
        self.on_estop     = on_estop
        self._stop        = threading.Event()
        self._thread      = threading.Thread(target=self._run, daemon=True)
        self._state_lock  = threading.Lock()
        self._estop_fired = False
        self._tof         = np.full((8, 8), TOF_MAX_RANGE_M, dtype=np.float32)
        self._imu         = np.zeros(6, dtype=np.float32)
        self._contact     = False
        self._last_rms    = 0.0

    def latest_observation(self) -> dict:
        with self._state_lock:
            return {"tof": self._tof.copy(), "imu": self._imu.copy()}

    @property
    def contact(self) -> bool:
        with self._state_lock:
            return self._contact

    @property
    def last_rms(self) -> float:
        with self._state_lock:
            return self._last_rms

    def start(self):
        self._thread.start()
        logger.info("SensorMonitor started on %s", self.port)

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
            logger.warning("SensorMonitor could not open %s: %s", self.port, e)
            return

        try:
            while not self._stop.is_set():
                # Drain the OS buffer to the most recent complete packet so the
                # recording loop always sees fresh data even if it polls slower
                # than 50Hz.
                raw = self._find_packet(ser)
                while ser.in_waiting >= PACKET_SIZE:
                    newer = self._find_packet(ser)
                    if newer is None:
                        break
                    raw = newer
                if raw is None:
                    continue

                (_, _, contact_flag, rms, estop_active,
                 ax, ay, az, gx, gy, gz, *tof_flat_and_checksum) = struct.unpack(PACKET_FMT, raw)
                tof_flat = tof_flat_and_checksum[:-1]

                tof_m = np.array(tof_flat, dtype=np.float32).reshape(8, 8)
                tof_m = np.where(tof_m == 0xFFFF, TOF_MAX_RANGE_M * 1000.0, tof_m) / 1000.0
                tof_m = np.clip(tof_m, 0.0, TOF_MAX_RANGE_M)

                with self._state_lock:
                    self._last_rms = rms
                    self._contact  = bool(contact_flag)
                    self._tof      = tof_m
                    self._imu      = np.array([ax, ay, az, gx, gy, gz], dtype=np.float32)

                if estop_active and self.on_estop and not self._estop_fired:
                    self._estop_fired = True
                    logger.warning("SensorMonitor: ESTOP received (contact_rms=%.2f)", rms)
                    self.on_estop()
        finally:
            ser.close()
```

---

## 6. udev Rules for Stable Device Names

Both the Feetech servo USB adapter and the Teensy 4.1 enumerate as `/dev/ttyACM*`. To avoid the
record/rollout scripts breaking depending on enumeration order, add stable symlinks keyed on each
device's USB serial number.

```
# /etc/udev/rules.d/99-roarm.rules
# Find serial numbers with: udevadm info -a -n /dev/ttyACM0 | grep '{serial}' | head -1
SUBSYSTEM=="tty", ATTRS{serial}=="<FEETECH_ADAPTER_SERIAL>", SYMLINK+="roarm_servo"
SUBSYSTEM=="tty", ATTRS{serial}=="<TEENSY_SERIAL>", SYMLINK+="roarm_teensy"
```

A copy of this file (with placeholders) lives at `firmware/tools/udev/99-roarm.rules` for reference.
The implementation plan includes a step to read each device's actual serial number and fill it in on
the RPi5, then `udevadm control --reload-rules && udevadm trigger`.

---

## 7. Dataset Schema (`scripts/record_roarm.py`)

### 7.1 New observation features

| Key | dtype | shape | Notes |
|---|---|---|---|
| `observation.tof` | float32 | (8, 8) | meters, row-major; invalid zones clamped to 4.0 (= "nothing in range") |
| `observation.imu` | float32 | (6,) | `[accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]`, m/s² and deg/s |

### 7.2 Existing features (unchanged from the AI/ML LeRobot plan)

| Key | dtype | shape | Notes |
|---|---|---|---|
| `observation.images.wrist` | uint8 | (480, 640, 3) | eye-in-hand USB camera |
| `observation.state` | float32 | (5,) | shoulder_pan, shoulder_lift, shoulder_lift_b, elbow_flex, gripper |
| `action` | float32 | (5,) | same joint ordering |
| `single_task` | string | — | per-episode task description (see §8) |

### 7.3 Recording loop integration

```python
from firmware.tools.sensor_listener import SensorMonitor

sensor_monitor = SensorMonitor(port="/dev/roarm_teensy", on_estop=lambda: robot.disconnect())
sensor_monitor.start()

# inside the per-frame callback used by record_loop():
def get_observation_extra():
    snap = sensor_monitor.latest_observation()
    return {"observation.tof": snap["tof"], "observation.imu": snap["imu"]}
```

`record_loop()` (from the AI/ML plan) calls `robot.get_observation()` for camera + joint state each
frame; `get_observation_extra()` is merged into that dict before `dataset.add_frame()`. This is a
single 30Hz polling point — the same "grab whatever's freshest" pattern already used for camera and
joint state, applied uniformly to the Teensy packet. No new threads or timers are introduced in the
recording script itself; `SensorMonitor` owns its own background thread (as `safety_listener` did).

If `sensor_monitor.contact` is `True` mid-episode, the existing safety-stop behavior (pause/abort
recording) applies exactly as it would have under the old safety listener — the `on_estop` callback
is unchanged in spirit, just sourced from the new packet.

---

## 8. Language Conditioning + Demo Plan

**2 task phrases, ~40 episodes each, 80 total** (within the 50-80 demo budget):

| Episodes | `single_task` | Object |
|---|---|---|
| 1–40 | `"pick the red cube and place it in the bin"` | red cube |
| 41–80 | `"pick the blue cube and place it in the bin"` | blue cube |

```python
TASK_DESCRIPTIONS = [
    "pick the red cube and place it in the bin",
    "pick the blue cube and place it in the bin",
]
EPISODES_PER_TASK = 40

def task_for_episode(episode_idx: int) -> str:
    return TASK_DESCRIPTIONS[episode_idx // EPISODES_PER_TASK]
```

**Within each 40-episode block**, vary:
- **Object position**: rotate through 4 staging positions (front-left, front-right, back-left,
  back-right of the bin) — 10 episodes per position.
- **Lighting/background**: change at least once every ~10 episodes (e.g., overhead light on/off,
  shift the backdrop) so the policy can't key off incidental scene cues.

The recording script should print the active phrase and a position/lighting reminder at the start of
each episode (derived from `episode_idx`), so the operator stages the scene correctly without needing
a separate checklist.

---

## 9. Ablation Study Plan (data already supports all of these)

Same eval set (fixed object positions/lighting, held out from training) across all variants:

1. **Baseline**: `observation.images.wrist` + `observation.state` only (drop `tof`/`imu` at train time)
2. **+ToF**: add `observation.tof`
3. **+ToF+IMU**: add `observation.imu`
4. **Language**: train on both phrases vs. train on phrase A only, eval on phrase B — tests whether
   `single_task` actually conditions object selection or the policy ignores it

Compare success rate (grasp success, placement success) per variant. This is friend's training-side
work; this spec only guarantees the dataset contains everything needed to run all four configs without
re-collection.

---

## 10. Superseded / Removed

| Item | Disposition |
|---|---|
| ESP32 DevKit V1 | Removed from architecture (spare hardware) |
| RPi Pico W | Unused spare, no role in this design |
| `firmware/src/safety_monitor/safety_main.cpp` | Deleted, replaced by `sensor_safety/sensor_main.cpp` |
| `firmware/src/safety_monitor/safety_comms.h` | Deleted, replaced by `sensor_safety/sensor_comms.h` |
| `firmware/tools/safety_listener.py` | Deleted, replaced by `firmware/tools/sensor_listener.py` |
| `firmware/platformio.ini` `[env:esp32dev]` | Removed, replaced by `[env:teensy41]` |
| `2026-06-05-ryan-hardware-prd.md` Task 8 | Superseded by this spec (sections 3-6) |

---

## 11. Open Risks / Verification Items

1. **Library portability**: confirm `stm32duino/STM32duino VL53L5CX` compiles under
   `platform = teensy, board = teensy41` (it's a generic Arduino `Wire`-based library, expected to work,
   but unverified on this MCU).
2. **I2C rewire bring-up**: re-verify IMU `WHO_AM_I` (0x6B) and a ToF distance reading on the new
   Teensy pins (18/19 + LPN=2/INT=3) before integrating into `sensor_main.cpp`.
3. **udev serial numbers**: must be read from the actual hardware on the RPi5 (placeholders in
   `99-roarm.rules` need real values).
4. **Throughput**: 168 bytes @ 50Hz = 8.4 KB/s — trivial for USB CDC, but confirm
   `SensorMonitor`'s drain loop keeps pace with a 30Hz consumer without unbounded backlog growth.
5. **ToF mounting**: physical placement of the VL53L5CX relative to the new wrist camera mount is a
   dependency of the (separate, already-planned) camera-mount task — not designed here, but the
   recording pipeline assumes it's wrist-mounted and roughly co-aligned with the camera's view axis.

---

## 12. Out of Scope

- ACT policy/model config changes to consume `observation.tof` / `observation.imu` (friend's scope;
  LeRobot auto-encodes additional observation keys, no schema blocker).
- `rpi5_inference/`, motor IDs, abandoned 250-byte telemetry — untouched, per existing PRD.
- Repurposing ESP32 or Pico W for any other function.
- Full 3-phrase / 30-50-demos-per-phrase language generalization floor — deferred to v3.
