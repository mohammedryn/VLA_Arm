from dataclasses import dataclass

from ..config import RobotConfig
from ..so_follower.config_so_follower import SOFollowerConfig


@RobotConfig.register_subclass("roarm_m2s_follower")
@dataclass
class RoArmM2SFollowerConfig(RobotConfig, SOFollowerConfig):
    """Configuration for the Waveshare RoArm M2-S (5-motor STS3215 arm).

    Motor layout:
        shoulder_pan    ID 1  DEGREES
        shoulder_lift   ID 2  DEGREES
        shoulder_lift_b ID 3  DEGREES  (mechanically coupled to ID 2 — always mirrors it)
        elbow_flex      ID 4  DEGREES
        gripper         ID 5  RANGE_0_100  (0=open, 100=closed)

    Port: /dev/ttyACM0 @ 1 Mbaud (SmartElex USB adapter → STS3215 bus)
    """
    pass
