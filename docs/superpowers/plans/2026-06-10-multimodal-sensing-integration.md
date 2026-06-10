# Multimodal Sensing Integration (ToF + IMU + Language) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the ESP32 safety-monitor firmware with a Teensy 4.1 sensor+safety co-processor that streams IMU+ToF data to the RPi5, and extend the LeRobot recording pipeline to record `observation.tof`, `observation.imu`, `observation.servo_load`, and rotating two-phrase task language from episode 1.

**Architecture:** Teensy 4.1 reads the ISM330DHCX IMU and VL53L5CX ToF over its default I2C bus and streams a 168-byte `SensorStatus_t` packet at 50Hz over USB serial (`/dev/roarm_teensy`). The RPi5's `SensorMonitor` (background thread) parses these packets; `RoArmM2SFollower.get_observation()` merges ToF/IMU/servo-load into the observation dict so `record_loop()` writes them to the dataset automatically. `record_roarm.py` rotates between two task phrases (40 episodes each) and prompts the operator to vary object position/lighting every 10 episodes.

**Tech Stack:** PlatformIO (`teensy41` / Arduino framework, C++17), Python 3 (`pyserial`, `numpy`, `pytest`), LeRobot (`roarm_m2s` robot plugin).

**Spec:** `docs/superpowers/specs/2026-06-10-multimodal-sensing-integration-design.md`

---

## File Structure

| File | Responsibility |
|---|---|
| `firmware/src/sensor_safety/sensor_comms.h` | `SensorStatus_t` packet struct + magic/checksum constants (shared contract with RPi5) |
| `firmware/src/sensor_safety/sensor_config.h` | Teensy pin map, I2C addresses, contact-oracle threshold, control-loop timing |
| `firmware/src/sensor_safety/sensor_main.cpp` | 50Hz `setup()`/`loop()` entry point — reads IMU+ToF, builds packet, writes to USB serial |
| `firmware/src/ism330dhcx_driver.cpp` | (modified) IMU driver — port to Teensy default `Wire`, include `sensor_config.h` |
| `firmware/src/tof_driver.cpp` | (modified) ToF driver — port to Teensy default `Wire`, include `sensor_config.h` |
| `firmware/platformio.ini` | (modified) replace `[env:esp32dev]` with `[env:teensy41]`, scoped `build_src_filter` |
| `firmware/tools/sensor_listener.py` | RPi5-side `SensorMonitor` thread — parses packets, exposes latest ToF/IMU/contact state, fires `on_estop` |
| `firmware/tools/test_sensor_listener.py` | Unit tests for packet parsing (`parse_packet`, `normalize_load`, checksum) |
| `firmware/tools/udev/99-roarm.rules` | udev rules creating `/dev/roarm_servo` and `/dev/roarm_teensy` symlinks |
| `scripts/roarm_recording_extras.py` | Pure helper functions: task-phrase rotation, staging-position rotation, episode setup messages |
| `scripts/test_roarm_recording_extras.py` | Unit tests for the above |
| `~/lerobot/src/lerobot/robots/roarm_m2s/config_roarm_m2s.py` | (modified, external repo) add `sensor_port` config field |
| `~/lerobot/src/lerobot/robots/roarm_m2s/roarm_m2s_follower.py` | (modified, external repo) wire `SensorMonitor` + servo-load into `get_observation()`/`observation_features` |
| `scripts/record_roarm.py` | Recording entrypoint — 80-episode (2x40) language-conditioned demo collection |

**Deleted:** `firmware/src/safety_monitor/` (both files), `firmware/tools/safety_listener.py`.

**Cross-repo note:** Tasks 1–7 and the `record_roarm.py` portion of Task 8 are entirely within `~/vla_rob` (this repo). The `roarm_m2s_follower.py`/`config_roarm_m2s.py` edits in Task 8 are in the `~/lerobot` clone created by `docs/superpowers/plans/2026-06-05-aiml-lerobot-integration.md` (Task 2). If that clone doesn't exist yet on the machine executing this plan, run that plan's Task 2 first — Task 8's diff applies on top of the files listed in this plan's "Key Technical Concepts" (already reproduced in full in Task 8 below, so no need to re-read that plan).

**Hardware-dependent tasks:** Task 4 (firmware flash + wiring bring-up) and Task 6 (udev rules) require physical access to the Teensy 4.1, IMU, and ToF sensor and cannot be completed by a subagent without that hardware. They are written as lab-manual procedures for whoever has the hardware on the bench. Tasks 1, 2, 3, 5, 7 are pure code/build tasks. Task 8 is mostly code but its final verification step requires the hardware from Task 4 to be connected.

---

### Task 1: Sensor packet protocol (`sensor_comms.h`, `sensor_config.h`)

**Files:**
- Create: `firmware/src/sensor_safety/sensor_comms.h`
- Create: `firmware/src/sensor_safety/sensor_config.h`
- Test: `firmware/src/sensor_safety/test_sensor_comms.cpp`

- [ ] **Step 1: Write the failing test**

```cpp
// firmware/src/sensor_safety/test_sensor_comms.cpp
#include <cassert>
#include <cstdio>
#include "sensor_comms.h"

int main() {
    assert(sizeof(SensorStatus_t) == 168);

    SensorStatus_t pkt = {};
    pkt.magic = SENSOR_PACKET_MAGIC;
    pkt.contact_flag = 1;
    pkt.estop_active = 0;
    pkt.tof_mm[0] = 0xFFFF;
    pkt.tof_mm[63] = 1234;

    assert(pkt.magic == 0xBEEFCAFEUL);
    assert(pkt.contact_flag == 1);
    assert(pkt.tof_mm[0] == 0xFFFF);
    assert(pkt.tof_mm[63] == 1234);

    printf("PASS: SensorStatus_t is %zu bytes\n", sizeof(SensorStatus_t));
    return 0;
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `g++ -std=c++17 -I firmware/src/sensor_safety firmware/src/sensor_safety/test_sensor_comms.cpp -o /tmp/test_sensor_comms`
Expected: FAIL — `fatal error: sensor_comms.h: No such file or directory`

- [ ] **Step 3: Create `sensor_comms.h`**

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

- [ ] **Step 4: Create `sensor_config.h`**

```cpp
// firmware/src/sensor_safety/sensor_config.h
#pragma once

// I2C bus (Teensy 4.1 default Wire: SDA=18, SCL=19)
#define IMU_I2C_ADDR            0x6B
#define TOF_I2C_ADDR            0x52
#define TOF_LPN                 2      // power enable, HIGH=powered
#define TOF_INT                 3      // data-ready, active LOW
#define TOF_UPDATE_HZ           15

// Contact oracle
#define CONTACT_WINDOW          8       // samples, ~38ms at 208Hz
#define SAFETY_CONTACT_THRESHOLD 3.5f   // deg/s RMS, calibrated value carried over

// Control loop
#define CONTROL_HZ              50
#define CONTROL_PERIOD_US       20000   // 20ms
```

- [ ] **Step 5: Run test to verify it passes**

Run: `g++ -std=c++17 -I firmware/src/sensor_safety firmware/src/sensor_safety/test_sensor_comms.cpp -o /tmp/test_sensor_comms && /tmp/test_sensor_comms`
Expected: `PASS: SensorStatus_t is 168 bytes`

- [ ] **Step 6: Commit**

```bash
git add firmware/src/sensor_safety/sensor_comms.h firmware/src/sensor_safety/sensor_config.h firmware/src/sensor_safety/test_sensor_comms.cpp
git commit -m "feat: add sensor packet protocol for Teensy 4.1 sensor co-processor"
```

---

### Task 2: Port IMU and ToF drivers to Teensy 4.1 default I2C bus

**Files:**
- Modify: `firmware/src/ism330dhcx_driver.cpp`
- Modify: `firmware/src/tof_driver.cpp`

These are Arduino-framework files and can't be compiled standalone with `g++`; the build is verified in Task 3 once `sensor_main.cpp` and the new `platformio.ini` env exist. This task is two mechanical, isolated edits.

- [ ] **Step 1: Edit `firmware/src/ism330dhcx_driver.cpp`**

Change line 2 from:
```cpp
#include "config.h"
```
to:
```cpp
#include "sensor_safety/sensor_config.h"
```

Change line 38 from:
```cpp
    Wire.begin(TOF_SDA, TOF_SCL);
```
to:
```cpp
    Wire.begin();
```

(Teensy 4.1's default `Wire` bus is fixed to pins 18/19 — no pin arguments needed or accepted.)

- [ ] **Step 2: Edit `firmware/src/tof_driver.cpp`**

Change line 3 from:
```cpp
#include "config.h"
```
to:
```cpp
#include "sensor_safety/sensor_config.h"
```

Change line 13 from:
```cpp
    Wire.begin(TOF_SDA, TOF_SCL);
```
to:
```cpp
    Wire.begin();
```

- [ ] **Step 3: Commit**

```bash
git add firmware/src/ism330dhcx_driver.cpp firmware/src/tof_driver.cpp
git commit -m "refactor: port IMU and ToF drivers to Teensy 4.1 default I2C bus"
```

---

### Task 3: Teensy 4.1 firmware entry point + PlatformIO environment

**Files:**
- Create: `firmware/src/sensor_safety/sensor_main.cpp`
- Modify: `firmware/platformio.ini`
- Delete: `firmware/src/safety_monitor/safety_main.cpp`
- Delete: `firmware/src/safety_monitor/safety_comms.h`

- [ ] **Step 1: Create `sensor_main.cpp`**

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

- [ ] **Step 2: Replace `firmware/platformio.ini`**

Replace the entire current contents:
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

with:
```ini
[env:teensy41]
platform      = teensy
board         = teensy41
framework     = arduino
build_flags   = -O2 -std=c++17
build_src_filter =
    -<*>
    +<ism330dhcx_driver.cpp>
    +<tof_driver.cpp>
    +<contact_oracle.cpp>
    +<sensor_safety/sensor_main.cpp>
lib_deps =
    Wire
    stm32duino/STM32duino VL53L5CX @ ^1.2.3
```

- [ ] **Step 3: Delete the old ESP32 safety-monitor firmware**

```bash
rm -rf firmware/src/safety_monitor
```

- [ ] **Step 4: Build the new environment**

Run: `cd firmware && pio run -e teensy41`
Expected: ends with `=== [SUCCESS] Took N.NN seconds ===`

If the build fails on the `STM32duino VL53L5CX` library against `teensy41` (this is the highest-risk dependency flagged in the spec's Open Risks), switch to the SparkFun fallback:

```bash
cd firmware && pio pkg uninstall -e teensy41 -l "stm32duino/STM32duino VL53L5CX"
cd firmware && pio pkg install -e teensy41 -l "sparkfun/SparkFun VL53L5CX Arduino Library"
```

then in `firmware/src/tof_driver.cpp`, change line 2 from:
```cpp
#include "vl53l5cx_class.h"
```
to:
```cpp
#include "SparkFun_VL53L5CX_Library.h"
```
and update the call sites (`VL53L5CX` → `SparkFun_VL53L5CX`, `init_sensor(addr)` → `begin(addr, Wire)`, `vl53l5cx_set_resolution`/`vl53l5cx_set_ranging_frequency_hz`/`vl53l5cx_set_sharpener_percent`/`vl53l5cx_start_ranging`/`vl53l5cx_check_data_ready`/`vl53l5cx_get_ranging_data` → the SparkFun library's equivalents (`setResolution`, `setRangingFrequency`, `setSharpenerPercent`, `startRanging`, `isDataReady`, `getRangingData`), and `VL53L5CX_ResultsData` field names per that library's struct. Re-run Step 4's build command and confirm `SUCCESS` before continuing.

- [ ] **Step 5: Commit**

```bash
git add firmware/src/sensor_safety/sensor_main.cpp firmware/platformio.ini
git rm -r firmware/src/safety_monitor
git commit -m "feat: add Teensy 4.1 sensor co-processor firmware, remove ESP32 safety monitor"
```

---

### Task 4: Hardware bring-up — wire Teensy 4.1, flash firmware, verify packet stream

This task requires physical access to the Teensy 4.1, ISM330DHCX breakout, and VL53L5CX breakout.

**Files:** none (hardware + flashing only)

- [ ] **Step 1: Power down and disconnect the old ESP32 wiring**

1. Unplug the ESP32 DevKit V1 from USB and from any servo-rail power.
2. Note and then disconnect the 4 sensor wires currently on ESP32 GPIO21 (I2C SDA), GPIO22 (I2C SCL), GPIO27 (ToF `LPN`), GPIO26 (ToF `INT`), plus the shared 3.3V and GND lines to both the ISM330DHCX and VL53L5CX breakouts.
3. Expected result: ESP32 is fully disconnected from both sensor breakouts; the breakouts are unpowered (LEDs off).

- [ ] **Step 2: Wire the sensors to the Teensy 4.1**

1. Connect ISM330DHCX `SDA` and VL53L5CX `SDA` together to Teensy pin **18 (SDA0)**.
2. Connect ISM330DHCX `SCL` and VL53L5CX `SCL` together to Teensy pin **19 (SCL0)**.
3. Connect VL53L5CX `LPN` to Teensy pin **2**.
4. Connect VL53L5CX `INT` to Teensy pin **3**.
5. Connect both breakouts' `VIN`/`VCC` to Teensy `3.3V`.
6. Connect both breakouts' `GND` to Teensy `GND`.
7. Add a 4.7kΩ pull-up resistor from pin 18 to 3.3V and another from pin 19 to 3.3V if your breakouts do not already have onboard I2C pull-ups (most VL53L5CX/ISM330DHCX breakout boards do — check the breakout's silkscreen/datasheet before adding external resistors).
8. Expected result: both breakouts' power LEDs light up once the Teensy is powered via USB in the next step.

- [ ] **Step 3: Connect Teensy 4.1 to the RPi5 via USB**

1. Plug the Teensy 4.1's USB-C/micro-USB port into a free USB port on the RPi5 (do **not** use the same unisolated 12V rail that powers the servos — the Teensy is powered entirely from USB, separate from the servo bus power supply).
2. Run on the RPi5: `ls /dev/ttyACM*`
3. Expected result: a new `/dev/ttyACM*` device appears compared to before plugging in (if the servo bus controller is also `/dev/ttyACM*`, you'll see two entries — note both, Task 6 disambiguates them permanently).

- [ ] **Step 4: Flash the new firmware**

Run: `cd firmware && pio run -e teensy41 -t upload`
Expected: ends with `=== [SUCCESS] Took N.NN seconds ===`. If `pio` reports it cannot find the Teensy bootloader, press the small white **PROGRAM** button on the Teensy 4.1 board once and re-run the command.

- [ ] **Step 5: Verify the raw packet stream**

Run (replace `/dev/ttyACM0` with the device noted in Step 3 — the one that newly appeared):
```bash
python3 -c "
import serial
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
data = ser.read(168 * 3)
print(len(data), data[:16].hex())
"
```
Expected: prints `504 ...` where the hex starts with the magic bytes `fecaefbe` (little-endian encoding of `0xBEEFCAFE`), repeating roughly every 168 bytes (`504 = 168 * 3`). If `len(data) < 504`, wait a few seconds after flashing (the Teensy reboots on upload) and re-run.

- [ ] **Step 6: Verify ToF and IMU readings respond to physical stimuli**

1. With the script from Step 5 running in a loop (wrap it in `while True:` temporarily, or just re-run it a few times), wave a hand in front of the VL53L5CX sensor.
2. Bytes 14-141 of each 168-byte packet are the 64 `tof_mm` `uint16` values (little-endian) — confirm they change when an object is moved closer/farther (you don't need to fully decode them yet; `sensor_listener.py` in Task 5 does that).
3. Tap the IMU breakout gently.
4. Bytes 9-32 are `accel[3]` and `gyro[3]` (8 floats total, 4 bytes each, little-endian) — confirm the bytes change when the board is tapped or rotated.
5. Expected result: both sensors visibly produce changing byte patterns in response to physical stimuli, confirming the I2C wiring and Teensy firmware are working end-to-end before moving on to Task 5's structured parser.

- [ ] **Step 7: No commit** — this task is hardware verification only, no files changed.

---

### Task 5: RPi5 sensor listener (`sensor_listener.py`)

**Files:**
- Create: `firmware/tools/sensor_listener.py`
- Test: `firmware/tools/test_sensor_listener.py`
- Delete: `firmware/tools/safety_listener.py`

This task separates pure packet-parsing logic (`parse_packet`, `normalize_load`, `compute_checksum`) — fully unit-testable without hardware — from the threaded `SensorMonitor` I/O wrapper.

- [ ] **Step 1: Write the failing tests**

```python
# firmware/tools/test_sensor_listener.py
import struct
import numpy as np
import pytest

from firmware.tools.sensor_listener import (
    PACKET_SIZE,
    SENSOR_MAGIC,
    TOF_MAX_RANGE_M,
    compute_checksum,
    normalize_load,
    parse_packet,
)

BODY_FMT = '<IIBfB3f3f64H'  # everything except the trailing checksum


def _build_packet(*, timestamp_us=0, contact_flag=0, contact_rms=0.0, estop_active=0,
                   accel=(0.0, 0.0, 0.0), gyro=(0.0, 0.0, 0.0), tof_mm=None) -> bytes:
    if tof_mm is None:
        tof_mm = [1000] * 64  # 1m everywhere
    body = struct.pack(
        BODY_FMT,
        SENSOR_MAGIC, timestamp_us, contact_flag, contact_rms, estop_active,
        *accel, *gyro, *tof_mm,
    )
    checksum = sum(body) & 0xFFFF
    return body + struct.pack('<H', checksum)


def test_packet_size():
    assert PACKET_SIZE == 168
    assert len(_build_packet()) == 168


def test_compute_checksum_matches_packet():
    raw = _build_packet(contact_rms=1.5)
    assert compute_checksum(raw[:-2]) == int.from_bytes(raw[-2:], 'little')


def test_parse_packet_valid_returns_expected_fields():
    raw = _build_packet(
        timestamp_us=12345,
        contact_flag=1,
        contact_rms=2.5,
        estop_active=0,
        accel=(0.1, 0.2, 9.8),
        gyro=(1.0, -1.0, 0.5),
    )
    result = parse_packet(raw)
    assert result["timestamp_us"] == 12345
    assert result["contact_flag"] is True
    assert result["estop_active"] is False
    assert result["contact_rms"] == pytest.approx(2.5)
    np.testing.assert_allclose(result["imu"], [0.1, 0.2, 9.8, 1.0, -1.0, 0.5], atol=1e-5)
    assert result["tof"].shape == (8, 8)
    np.testing.assert_allclose(result["tof"], 1.0, atol=1e-5)  # 1000mm -> 1.0m


def test_parse_packet_invalid_tof_zone_clamps_to_max_range():
    tof_mm = [1000] * 64
    tof_mm[0] = 0xFFFF  # invalid sentinel
    raw = _build_packet(tof_mm=tof_mm)
    result = parse_packet(raw)
    assert result["tof"][0, 0] == pytest.approx(TOF_MAX_RANGE_M)


def test_parse_packet_bad_checksum_raises():
    raw = _build_packet()
    corrupted = raw[:-1] + bytes([raw[-1] ^ 0xFF])
    with pytest.raises(ValueError, match="checksum"):
        parse_packet(corrupted)


def test_parse_packet_wrong_size_raises():
    with pytest.raises(ValueError, match="168"):
        parse_packet(b"\x00" * 100)


@pytest.mark.parametrize("raw,expected", [
    (0,    0.0),
    (500,  0.5),
    (1000, 1.0),
    (0x400 | 500, -0.5),  # direction bit set
])
def test_normalize_load(raw, expected):
    assert normalize_load(raw) == pytest.approx(expected)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd ~/vla_rob && python3 -m pytest firmware/tools/test_sensor_listener.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'firmware.tools.sensor_listener'` (or `ImportError`)

- [ ] **Step 3: Create `sensor_listener.py`**

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


def compute_checksum(body: bytes) -> int:
    """Sum of all bytes except the trailing 2-byte checksum field, mod 65536."""
    return sum(body) & 0xFFFF


def normalize_load(raw: int) -> float:
    """STS3215 Present_Load: bit10=direction, bits0-9=magnitude (0-1000 -> 0.0-1.0)."""
    magnitude = (raw & 0x3FF) / 1000.0
    sign = -1.0 if (raw & 0x400) else 1.0
    return sign * magnitude


def parse_packet(raw: bytes) -> dict:
    """Parse and validate a 168-byte SensorStatus_t packet.

    Returns a dict with keys: timestamp_us, contact_flag, contact_rms,
    estop_active, imu (6,) float32, tof (8,8) float32 in meters.
    Raises ValueError if the packet is the wrong size or fails its checksum.
    """
    if len(raw) != PACKET_SIZE:
        raise ValueError(f"expected {PACKET_SIZE} bytes, got {len(raw)}")

    computed = compute_checksum(raw[:-2])
    received = int.from_bytes(raw[-2:], 'little')
    if computed != received:
        raise ValueError(f"checksum mismatch: computed {computed}, received {received}")

    (_, timestamp_us, contact_flag, rms, estop_active,
     ax, ay, az, gx, gy, gz, *rest) = struct.unpack(PACKET_FMT, raw)
    tof_raw = rest[:64]

    tof_m = np.array(tof_raw, dtype=np.float32).reshape(8, 8)
    tof_m = np.where(tof_m == 0xFFFF, TOF_MAX_RANGE_M * 1000.0, tof_m) / 1000.0
    tof_m = np.clip(tof_m, 0.0, TOF_MAX_RANGE_M)

    return {
        "timestamp_us": timestamp_us,
        "contact_flag": bool(contact_flag),
        "contact_rms": rms,
        "estop_active": bool(estop_active),
        "imu": np.array([ax, ay, az, gx, gy, gz], dtype=np.float32),
        "tof": tof_m,
    }


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
            try:
                parse_packet(pkt)
                return pkt
            except ValueError:
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

                result = parse_packet(raw)

                with self._state_lock:
                    self._last_rms = result["contact_rms"]
                    self._contact  = result["contact_flag"]
                    self._tof      = result["tof"]
                    self._imu      = result["imu"]

                if result["estop_active"] and self.on_estop and not self._estop_fired:
                    self._estop_fired = True
                    logger.warning("SensorMonitor: ESTOP received (contact_rms=%.2f)", result["contact_rms"])
                    self.on_estop()
        finally:
            ser.close()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd ~/vla_rob && python3 -m pytest firmware/tools/test_sensor_listener.py -v`
Expected: all 8 tests `PASSED`

- [ ] **Step 5: Delete the old ESP32 safety listener**

```bash
rm firmware/tools/safety_listener.py
```

- [ ] **Step 6: Commit**

```bash
git add firmware/tools/sensor_listener.py firmware/tools/test_sensor_listener.py
git rm firmware/tools/safety_listener.py
git commit -m "feat: add Teensy sensor packet listener with ToF/IMU/checksum unit tests"
```

---

### Task 6: udev rules for stable `/dev/roarm_*` device names

This task requires the Teensy 4.1 (Task 4) and the existing Feetech servo USB adapter both connected to the RPi5.

**Files:**
- Create: `firmware/tools/udev/99-roarm.rules`

- [ ] **Step 1: Find each device's serial number**

With both the Teensy 4.1 and the servo bus adapter plugged into the RPi5, run:
```bash
for dev in /dev/ttyACM*; do
    echo "$dev:"
    udevadm info -a -n "$dev" | grep -m1 'ATTRS{serial}'
done
```
Expected: two lines, one per device, each like:
```
/dev/ttyACM0:
    ATTRS{serial}=="12345678"
/dev/ttyACM1:
    ATTRS{serial}=="ABCD1234EF"
```
Note which serial belongs to which device — unplug the Teensy alone and re-run the command; whichever `/dev/ttyACM*` disappears is the Teensy's serial number. The remaining one is the servo bus adapter. Plug the Teensy back in.

- [ ] **Step 2: Create the udev rules file**

Replace `TEENSY_SERIAL_HERE` and `SERVO_SERIAL_HERE` with the two serial numbers found in Step 1.

```bash
# firmware/tools/udev/99-roarm.rules
# Stable device names for the RoArm M2-S project.
# Install with: sudo cp firmware/tools/udev/99-roarm.rules /etc/udev/rules.d/
#               sudo udevadm control --reload-rules && sudo udevadm trigger

SUBSYSTEM=="tty", ATTRS{serial}=="TEENSY_SERIAL_HERE", SYMLINK+="roarm_teensy"
SUBSYSTEM=="tty", ATTRS{serial}=="SERVO_SERIAL_HERE", SYMLINK+="roarm_servo"
```

Write this file to `firmware/tools/udev/99-roarm.rules` with your actual serial numbers substituted.

- [ ] **Step 3: Install and reload the rules**

```bash
sudo cp firmware/tools/udev/99-roarm.rules /etc/udev/rules.d/99-roarm.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```
Expected: no output (success is silent).

- [ ] **Step 4: Verify the symlinks exist**

```bash
ls -l /dev/roarm_teensy /dev/roarm_servo
```
Expected: both resolve via `->` to `/dev/ttyACM0`/`/dev/ttyACM1` (or similar), e.g.:
```
lrwxrwxrwx 1 root root 7 ... /dev/roarm_servo -> ttyACM1
lrwxrwxrwx 1 root root 7 ... /dev/roarm_teensy -> ttyACM0
```

- [ ] **Step 5: Commit**

```bash
git add firmware/tools/udev/99-roarm.rules
git commit -m "feat: add udev rules for stable /dev/roarm_servo and /dev/roarm_teensy"
```

---

### Task 7: Recording-loop helpers — language and scene rotation

**Files:**
- Create: `scripts/roarm_recording_extras.py`
- Test: `scripts/test_roarm_recording_extras.py`

Pure functions, no hardware dependency — fully unit-testable.

- [ ] **Step 1: Write the failing tests**

```python
# scripts/test_roarm_recording_extras.py
import pytest

from scripts.roarm_recording_extras import (
    EPISODES_PER_TASK,
    POSITIONS,
    TASK_DESCRIPTIONS,
    episode_setup_message,
    position_for_episode,
    task_for_episode,
)


def test_task_for_episode_first_block_is_phrase_a():
    assert task_for_episode(0) == TASK_DESCRIPTIONS[0]
    assert task_for_episode(EPISODES_PER_TASK - 1) == TASK_DESCRIPTIONS[0]


def test_task_for_episode_second_block_is_phrase_b():
    assert task_for_episode(EPISODES_PER_TASK) == TASK_DESCRIPTIONS[1]
    assert task_for_episode(2 * EPISODES_PER_TASK - 1) == TASK_DESCRIPTIONS[1]


def test_task_for_episode_out_of_range_raises():
    with pytest.raises(ValueError, match="exceeds"):
        task_for_episode(2 * EPISODES_PER_TASK)


def test_position_for_episode_rotates_every_ten_within_block():
    assert position_for_episode(0) == POSITIONS[0]
    assert position_for_episode(9) == POSITIONS[0]
    assert position_for_episode(10) == POSITIONS[1]
    assert position_for_episode(39) == POSITIONS[3]


def test_position_for_episode_wraps_for_second_task_block():
    # episode 40 is the first episode of phrase B, position rotation restarts
    assert position_for_episode(40) == POSITIONS[0]


def test_episode_setup_message_flags_lighting_change_at_block_starts():
    msg0 = episode_setup_message(0)
    msg1 = episode_setup_message(1)
    assert "vary lighting" in msg0
    assert "vary lighting" not in msg1
    assert TASK_DESCRIPTIONS[0] in msg0
    assert POSITIONS[0] in msg0
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd ~/vla_rob && python3 -m pytest scripts/test_roarm_recording_extras.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'scripts.roarm_recording_extras'`

- [ ] **Step 3: Create `roarm_recording_extras.py`**

```python
# scripts/roarm_recording_extras.py
"""Helper functions for language-phrase and scene-variation rotation in record_roarm.py."""

TASK_DESCRIPTIONS = [
    "pick the red cube and place it in the bin",
    "pick the blue cube and place it in the bin",
]
EPISODES_PER_TASK = 40

POSITIONS = ["front-left", "front-right", "back-left", "back-right"]
EPISODES_PER_POSITION = 10


def task_for_episode(episode_idx: int) -> str:
    """Return the single_task language phrase for a 0-indexed episode number."""
    task_idx = episode_idx // EPISODES_PER_TASK
    if task_idx >= len(TASK_DESCRIPTIONS):
        raise ValueError(
            f"episode_idx {episode_idx} exceeds planned "
            f"{len(TASK_DESCRIPTIONS) * EPISODES_PER_TASK} episodes"
        )
    return TASK_DESCRIPTIONS[task_idx]


def position_for_episode(episode_idx: int) -> str:
    """Return the staging position for a 0-indexed episode number.

    Position rotates through all 4 positions every EPISODES_PER_TASK episodes,
    restarting at the beginning of each task-phrase block.
    """
    block_idx = episode_idx % EPISODES_PER_TASK
    return POSITIONS[block_idx // EPISODES_PER_POSITION]


def episode_setup_message(episode_idx: int) -> str:
    """Operator-facing message printed before recording each episode."""
    task = task_for_episode(episode_idx)
    position = position_for_episode(episode_idx)
    msg = f'Episode {episode_idx + 1}: "{task}" -- stage object at {position}'
    if episode_idx % EPISODES_PER_POSITION == 0:
        msg += " -- vary lighting/background now"
    return msg
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd ~/vla_rob && python3 -m pytest scripts/test_roarm_recording_extras.py -v`
Expected: all 6 tests `PASSED`

- [ ] **Step 5: Commit**

```bash
git add scripts/roarm_recording_extras.py scripts/test_roarm_recording_extras.py
git commit -m "feat: add task-phrase and scene-rotation helpers for recording script"
```

---

### Task 8: Wire ToF/IMU/servo-load into RoArmM2SFollower and create record_roarm.py

**Files:**
- Modify: `~/lerobot/src/lerobot/robots/roarm_m2s/config_roarm_m2s.py` (external repo, see "Cross-repo note" above)
- Modify: `~/lerobot/src/lerobot/robots/roarm_m2s/roarm_m2s_follower.py` (external repo)
- Create: `scripts/record_roarm.py`

**Import note:** `roarm_m2s_follower.py` imports `from firmware.tools.sensor_listener import SensorMonitor, normalize_load`. This mirrors the existing import of `firmware.tools.safety_listener.SafetyMonitor` in the rollout script from `docs/superpowers/plans/2026-06-05-aiml-lerobot-integration.md` (Task 6) — whatever mechanism makes `firmware.tools.*` importable for that script (e.g. running with `~/vla_rob` on `PYTHONPATH`, or `pip install -e ~/vla_rob`) makes it importable here too. No new setup is introduced by this task.

- [ ] **Step 1: Add `sensor_port` to `RoArmM2SFollowerConfig`**

In `~/lerobot/src/lerobot/robots/roarm_m2s/config_roarm_m2s.py`, the current full content is:
```python
from dataclasses import dataclass

from ..config import RobotConfig
from ..so_follower.config_so_follower import SOFollowerConfig


@RobotConfig.register_subclass("roarm_m2s_follower")
@dataclass
class RoArmM2SFollowerConfig(RobotConfig, SOFollowerConfig):
    """Configuration for the Waveshare RoArm M2-S (5-motor STS3215 arm)."""
    pass
```

Replace the class body so the file reads:
```python
from dataclasses import dataclass

from ..config import RobotConfig
from ..so_follower.config_so_follower import SOFollowerConfig


@RobotConfig.register_subclass("roarm_m2s_follower")
@dataclass
class RoArmM2SFollowerConfig(RobotConfig, SOFollowerConfig):
    """Configuration for the Waveshare RoArm M2-S (5-motor STS3215 arm)."""
    sensor_port: str = "/dev/roarm_teensy"
```

- [ ] **Step 2: Extend `RoArmM2SFollower` with sensor monitor, observation features, and observation merging**

In `~/lerobot/src/lerobot/robots/roarm_m2s/roarm_m2s_follower.py`, add two imports at the top, alongside the existing imports:
```python
import numpy as np

from firmware.tools.sensor_listener import SensorMonitor, normalize_load
```

In `__init__`, after the existing line `self.cameras = make_cameras_from_configs(config.cameras)`, add:
```python
        self._sensor_monitor = SensorMonitor(
            port=self.config.sensor_port,
            on_estop=lambda: self.disconnect(),
        )
```

Add three new methods to the class (after `__init__`, before `calibrate`):
```python
    def connect(self, calibrate: bool = True) -> None:
        super().connect(calibrate=calibrate)
        self._sensor_monitor.start()

    def disconnect(self) -> None:
        self._sensor_monitor.stop()
        super().disconnect()

    @property
    def observation_features(self) -> dict:
        features = dict(super().observation_features)
        features["tof"] = (8, 8)
        features["imu"] = (6,)
        features["servo_load"] = (5,)
        return features

    def get_observation(self) -> dict:
        obs = super().get_observation()
        snap = self._sensor_monitor.latest_observation()
        obs["tof"] = snap["tof"]
        obs["imu"] = snap["imu"]
        raw_load = self.bus.sync_read("Present_Load")
        obs["servo_load"] = np.array(
            [normalize_load(raw_load[m]) for m in self.bus.motors], dtype=np.float32
        )
        return obs
```

**Note on `connect`/`disconnect` signatures:** the `calibrate: bool = True` parameter on `connect()` matches the common LeRobot `Robot.connect(calibrate=True)` convention. If the installed LeRobot version's `SOFollower.connect()`/`disconnect()` use a different signature, adjust the override's parameters to match — the only required addition in each is the `self._sensor_monitor.start()` / `self._sensor_monitor.stop()` call. Check with:
```bash
python3 -c "import inspect; from lerobot.robots.so_follower.so_follower import SOFollower; print(inspect.signature(SOFollower.connect)); print(inspect.signature(SOFollower.disconnect))"
```

- [ ] **Step 3: Create `record_roarm.py`**

```python
#!/usr/bin/env python3
# ~/vla_rob/scripts/record_roarm.py
"""
Record teleoperation demonstrations for RoArm M2-S using a USB gamepad.

Records 2 task phrases x 40 episodes each (80 total), rotating the staging
position every 10 episodes within each phrase block. Each frame includes
observation.tof, observation.imu, and observation.servo_load in addition to
the wrist camera image and joint state.

Requirements:
    pip install inputs   # gamepad library used by LeRobot's GamepadTeleop

Usage:
    python scripts/record_roarm.py \
        --follower_port /dev/roarm_servo \
        --sensor_port /dev/roarm_teensy \
        --repo_id YOUR_HF_USERNAME/roarm_pickplace_v2

Gamepad controls (standard layout):
    Left stick  -> shoulder_pan + shoulder_lift
    Right stick -> elbow_flex
    R2 trigger  -> gripper close
    L2 trigger  -> gripper open

Recording controls:
    Right arrow (d-pad) -> save episode, move to next
    Left arrow  (d-pad) -> discard episode, re-record
    Start button        -> stop recording early

If the Teensy sensor co-processor reports a hard contact (ESTOP), the robot
disconnects automatically and this script exits with an error message --
re-run it to resume (already-saved episodes are preserved in the dataset).
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

from scripts.roarm_recording_extras import TASK_DESCRIPTIONS, EPISODES_PER_TASK, episode_setup_message, task_for_episode

FPS = 30
EPISODE_TIME_SEC = 30   # seconds per demo
RESET_TIME_SEC   = 10   # pause between demos
NUM_EPISODES     = len(TASK_DESCRIPTIONS) * EPISODES_PER_TASK  # 80


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--follower_port", default="/dev/roarm_servo")
    parser.add_argument("--sensor_port",   default="/dev/roarm_teensy")
    parser.add_argument("--repo_id",       required=True, help="e.g. yourname/roarm_pickplace_v2")
    parser.add_argument("--num_episodes",  type=int, default=NUM_EPISODES)
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
        sensor_port=args.sensor_port,
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

    for key in ("observation.tof", "observation.imu", "observation.servo_load"):
        if key not in dataset.features:
            raise RuntimeError(
                f"{key} missing from dataset.features after LeRobotDataset.create(). "
                f"Got: {sorted(dataset.features.keys())}. "
                f"Check RoArmM2SFollower.observation_features and the installed LeRobot "
                f"version's create_initial_features/aggregate_pipeline_dataset_features "
                f"naming convention for non-image observation keys, then update the key "
                f"names used here and in roarm_m2s_follower.observation_features to match."
            )

    teleop.connect()
    follower.connect()

    listener, events = init_keyboard_listener()

    try:
        episode_idx = 0
        while episode_idx < args.num_episodes and not events["stop_recording"]:
            print(episode_setup_message(episode_idx))
            input("Press ENTER when the scene is staged and ready...")

            task = task_for_episode(episode_idx)
            log_say(f"Recording episode {episode_idx + 1} of {args.num_episodes}: {task}")

            record_loop(
                robot=follower,
                events=events,
                fps=FPS,
                teleop=teleop,
                dataset=dataset,
                control_time_s=EPISODE_TIME_SEC,
                single_task=task,
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
                log_say(f"Reset the scene. Resuming in {RESET_TIME_SEC} seconds.")
                record_loop(
                    robot=follower,
                    events=events,
                    fps=FPS,
                    teleop=teleop,
                    control_time_s=RESET_TIME_SEC,
                    single_task=task,
                    display_data=True,
                )
    except RuntimeError as e:
        # SensorMonitor.on_estop() calls follower.disconnect(), which causes the
        # next bus operation in record_loop() to raise. Treat this as a clean abort.
        log_say(f"Recording stopped: {e}")
        print(
            "Recording aborted, likely due to a Teensy ESTOP (hard contact detected). "
            f"{episode_idx} episode(s) were saved. Re-run this script to continue "
            "recording into the same dataset."
        )
        return
    finally:
        listener.stop()
        if follower.is_connected:
            follower.disconnect()
        teleop.disconnect()


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Verify dataset feature names against the installed LeRobot version**

Hardware required: Teensy 4.1 (Task 4) connected via `/dev/roarm_teensy`, servo bus connected via `/dev/roarm_servo`.

Run a short dry run:
```bash
cd ~/vla_rob
python3 scripts/record_roarm.py --repo_id local/roarm_sensor_check --num_episodes 1
```
Expected: the script either prints `Episode 1: "pick the red cube..." -- stage object at front-left -- vary lighting/background now` and waits for ENTER (success path — the `dataset.features` assertion passed), or raises the `RuntimeError` from Step 3 listing the actual feature key names. If the latter, update the key names in `RoArmM2SFollower.observation_features` (Step 2) and the assertion loop (Step 3) to match the printed `dataset.features` keys, then re-run.

You can press Ctrl+C after confirming the prompt appears — a 1-episode recording isn't required to validate this wiring, only that `LeRobotDataset.create()` accepted the new features.

- [ ] **Step 5: Commit**

The `~/lerobot` changes are in a separate repository — commit there:
```bash
cd ~/lerobot
git add src/lerobot/robots/roarm_m2s/config_roarm_m2s.py src/lerobot/robots/roarm_m2s/roarm_m2s_follower.py
git commit -m "feat: add ToF, IMU, and servo-load observations to RoArm M2-S follower"
```

Then in `~/vla_rob`:
```bash
cd ~/vla_rob
git add scripts/record_roarm.py
git commit -m "feat: add language-conditioned recording script with ToF/IMU/servo-load"
```

---

## Self-Review

**Spec coverage:**
- Architecture (Teensy 4.1 sole co-proc, ESP32/Pico W removed) — Tasks 1-3, 5
- `SensorStatus_t` packet (168 bytes) — Task 1
- Hardware/wiring changes — Task 4
- Driver porting (`Wire.begin()`, `sensor_config.h`) — Task 2
- `sensor_main.cpp` / `platformio.ini` (with corrected `build_src_filter`) — Task 3
- `sensor_listener.py` / `SensorMonitor` — Task 5
- udev rules — Task 6
- Dataset schema (`observation.tof`, `observation.imu`, `observation.servo_load`) — Task 8
- Language conditioning (`TASK_DESCRIPTIONS`, `EPISODES_PER_TASK=40`, `task_for_episode`) — Tasks 7-8
- Scene rotation (position every 10 episodes, lighting reminder) — Task 7
- Servo load normalization (`normalize_load`) — Tasks 5, 8
- `firmware/src/safety_monitor/` and `firmware/tools/safety_listener.py` removal — Tasks 3, 5
- Ablation plan, Out of Scope, Open Risks — informational sections of the spec, not separate implementation tasks; the VL53L5CX fallback risk is addressed in Task 3 Step 4, the `SensorMonitor._estop_fired` one-shot-latch behavior is implemented as designed in Task 5, and the `normalize_load` bit-layout is unit-tested in Task 5 and exercised live in Task 8 Step 4.

**Placeholder scan:** no `TBD`/`TODO`/"add error handling"/"similar to Task N" patterns remain; every code step has complete, runnable content.

**Type/name consistency:** `SensorStatus_t`, `SENSOR_PACKET_MAGIC`, `PACKET_FMT`/`PACKET_SIZE`, `parse_packet`, `normalize_load`, `SensorMonitor.latest_observation()` returning `{"tof", "imu"}`, `task_for_episode`/`position_for_episode`/`episode_setup_message`, `TASK_DESCRIPTIONS`/`EPISODES_PER_TASK`, and `RoArmM2SFollowerConfig.sensor_port` are each defined once and referenced identically across all later tasks.

**Fixed during planning:** the spec's original `build_src_filter` (`+<*> -<main.cpp> -<safety_monitor/*> +<sensor_safety/sensor_main.cpp>`) would have compiled the abandoned `comms.cpp`/`servo_bus.cpp`/`waypoint_interp.cpp`/`safety_layer.cpp` (which `#include "config.h"`, ESP32-specific) into the `teensy41` build. Both the spec (§4.6) and Task 3 here now use an opt-in filter (`-<*>` plus four explicit `+<...>` includes).
