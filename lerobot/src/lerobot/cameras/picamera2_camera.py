from dataclasses import dataclass
from picamera2 import Picamera2
import numpy as np
import threading
import time


@dataclass
class Picamera2CameraConfig:
    width: int = 640
    height: int = 480
    fps: int = 30
    af_mode: int = 2    # AfMode=2 = continuous autofocus (confirmed working on IMX708)
    af_range: int = 2   # AfRange=2 = macro range (gripper is 11.5cm from tip)


class Picamera2Camera:
    """
    LeRobot-compatible camera driver for Raspberry Pi Camera Module 3 (IMX708, MIPI CSI).

    cv2.VideoCapture via v4l2 is unreliable on RPi5 Ubuntu 24.04 with the RPi
    libcamera fork v0.5.2 + picamera2. Use this class instead of OpenCVCamera.

    Hardware: wrist-mount, 11.5 cm from gripper tip.
    """

    def __init__(self, config: Picamera2CameraConfig, name: str = "wrist"):
        self.config = config
        self.name = name
        self._cam = None
        self._lock = threading.Lock()

    def connect(self):
        self._cam = Picamera2()
        self._cam.configure(self._cam.create_video_configuration(
            main={"size": (self.config.width, self.config.height), "format": "RGB888"},
            controls={
                "AfMode": self.config.af_mode,
                "AfRange": self.config.af_range,
                "FrameRate": float(self.config.fps),
            },
        ))
        self._cam.start()
        time.sleep(1.0)  # allow autofocus to converge before first frame

    def read(self) -> np.ndarray:
        with self._lock:
            return self._cam.capture_array()

    def disconnect(self):
        if self._cam:
            self._cam.stop()
            self._cam = None

    @property
    def observation_feature(self):
        return {
            "shape": (self.config.height, self.config.width, 3),
            "dtype": "uint8",
        }
