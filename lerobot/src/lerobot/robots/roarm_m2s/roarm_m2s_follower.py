import logging
from functools import cached_property

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.types import RobotAction

from ..robot import Robot
from ..so_follower.so_follower import SOFollower
from ..utils import ensure_safe_goal_position
from .config_roarm_m2s import RoArmM2SFollowerConfig

logger = logging.getLogger(__name__)


def _make_cameras(camera_configs: dict) -> dict:
    """Camera factory that handles Picamera2CameraConfig in addition to LeRobot's built-ins."""
    cameras = {}
    for name, cfg in camera_configs.items():
        try:
            from lerobot.cameras.picamera2_camera import Picamera2CameraConfig, Picamera2Camera
            if isinstance(cfg, Picamera2CameraConfig):
                cameras[name] = Picamera2Camera(cfg, name=name)
                continue
        except ImportError:
            pass
        # Fall through to LeRobot's standard camera factory for all other config types
        from lerobot.cameras import make_cameras_from_configs
        cameras.update(make_cameras_from_configs({name: cfg}))
    return cameras


class RoArmM2SFollower(SOFollower):
    """
    Waveshare RoArm M2-S — 5-motor STS3215 arm with mechanically coupled shoulder.

    Differences from SO-100:
      - 5 motors instead of 6 (no wrist_roll)
      - shoulder_lift (ID 2) and shoulder_lift_b (ID 3) are on the same joint;
        send_action() always overwrites shoulder_lift_b with shoulder_lift's value
      - gripper is ID 5 on RANGE_0_100 norm (not DEGREES)
    """

    config_class = RoArmM2SFollowerConfig
    name = "roarm_m2s_follower"

    def __init__(self, config: RoArmM2SFollowerConfig):
        # Call Robot.__init__ directly — SOFollower.__init__ would build a
        # 6-motor SO-100 bus that we'd have to replace anyway.
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
        self.cameras = _make_cameras(config.cameras)

    def calibrate(self) -> None:
        """Override SOFollower.calibrate() — removes hardcoded wrist_roll logic."""
        if self.calibration:
            user_input = input(
                f"Press ENTER to use saved calibration for '{self.id}', "
                "or type 'c' + ENTER to run new calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info("Writing saved calibration for %s to motors", self.id)
                self.bus.write_calibration(self.calibration)
                return

        logger.info("Running calibration for %s", self)
        self.bus.disable_torque()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input("Move RoArm M2-S to the MIDDLE of each joint's range, then press ENTER...")
        homing_offsets = self.bus.set_half_turn_homings()

        input(
            "Now move ALL joints sequentially through their FULL ranges of motion.\n"
            "Press ENTER when done..."
        )
        range_mins, range_maxes = self.bus.record_ranges_of_motion(list(self.bus.motors.keys()))

        self.calibration = {
            motor: MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )
            for motor, m in self.bus.motors.items()
        }

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print(f"Calibration saved to {self.calibration_fpath}")

    def send_action(self, action: RobotAction) -> RobotAction:
        """Send joint positions, enforcing shoulder_lift_b = shoulder_lift (coupled joint)."""
        goal_pos = {
            key.removesuffix(".pos"): val
            for key, val in action.items()
            if key.endswith(".pos")
        }

        # Coupled shoulder: ID 3 must always mirror ID 2
        if "shoulder_lift" in goal_pos:
            goal_pos["shoulder_lift_b"] = goal_pos["shoulder_lift"]

        if self.config.max_relative_target is not None:
            present_pos = self.bus.sync_read("Present_Position")
            goal_present_pos = {
                key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()
            }
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        self.bus.sync_write("Goal_Position", goal_pos)
        return {f"{motor}.pos": val for motor, val in goal_pos.items()}
