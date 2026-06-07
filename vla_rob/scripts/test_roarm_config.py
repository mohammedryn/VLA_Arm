"""
Smoke-test for the RoArm M2-S robot config.
Run BEFORE implementing to confirm it fails (ModuleNotFoundError),
then again after install_roarm_config.sh to confirm it passes.

Does NOT connect to hardware — only tests Python-level config correctness.
"""
from lerobot.robots.roarm_m2s import RoArmM2SFollower, RoArmM2SFollowerConfig

config = RoArmM2SFollowerConfig(port="/dev/ttyACM0", id="roarm_test")
robot = RoArmM2SFollower(config)

assert robot.name == "roarm_m2s_follower", f"wrong name: {robot.name}"
assert list(robot.bus.motors.keys()) == [
    "shoulder_pan", "shoulder_lift", "shoulder_lift_b", "elbow_flex", "gripper"
], f"wrong motor names: {list(robot.bus.motors.keys())}"
assert robot.bus.motors["shoulder_pan"].id    == 1
assert robot.bus.motors["shoulder_lift"].id   == 2
assert robot.bus.motors["shoulder_lift_b"].id == 3
assert robot.bus.motors["elbow_flex"].id      == 4
assert robot.bus.motors["gripper"].id         == 5

print("PASS: robot config correct")
