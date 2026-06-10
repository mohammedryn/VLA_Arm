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
