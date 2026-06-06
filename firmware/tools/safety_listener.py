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
        self.port          = port
        self.baud          = baud
        self.on_estop      = on_estop
        self._stop         = threading.Event()
        self._thread       = threading.Thread(target=self._run, daemon=True)
        self._state_lock   = threading.Lock()
        self._estop_fired  = False
        self._last_rms     = 0.0
        self._contact      = False

    @property
    def last_rms(self) -> float:
        with self._state_lock:
            return self._last_rms

    @property
    def contact(self) -> bool:
        with self._state_lock:
            return self._contact

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
                with self._state_lock:
                    self._last_rms = rms
                    self._contact  = bool(contact_flag)
                if estop_active and self.on_estop and not self._estop_fired:
                    self._estop_fired = True
                    logger.warning("SafetyMonitor: ESTOP received (contact_rms=%.2f)", rms)
                    self.on_estop()
        finally:
            ser.close()
