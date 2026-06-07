#!/usr/bin/env python3
"""
Record teleoperation demonstrations for RoArm M2-S using a USB gamepad.

Camera: Raspberry Pi Camera Module 3 (IMX708, MIPI CSI) via picamera2.
        DO NOT use OpenCVCamera — cv2.VideoCapture is unreliable on RPi5 Ubuntu 24.04
        with the RPi libcamera fork v0.5.2.

Requirements:
    pip install inputs   # gamepad library used by LeRobot's GamepadTeleop

Usage:
    python scripts/record_roarm.py \\
        --follower_port /dev/ttyACM0 \\
        --repo_id YOUR_HF_USERNAME/roarm_pickplace_v1 \\
        --num_episodes 50

Gamepad controls (standard layout):
    Left stick   → shoulder_pan + shoulder_lift
    Right stick  → elbow_flex
    R2 trigger   → gripper close
    L2 trigger   → gripper open

Recording controls:
    D-pad right (→) → save episode, advance to next
    D-pad left  (←) → discard episode, re-record
    Start button    → stop recording early
"""

import argparse

from lerobot.cameras.picamera2_camera import Picamera2CameraConfig
from lerobot.common.control_utils import init_keyboard_listener
from lerobot.datasets import LeRobotDataset, aggregate_pipeline_dataset_features, create_initial_features
from lerobot.processor import make_default_processors
from lerobot.robots.roarm_m2s import RoArmM2SFollower, RoArmM2SFollowerConfig
from lerobot.scripts.lerobot_record import record_loop
from lerobot.teleoperators.gamepad import GamepadTeleop, GamepadTeleopConfig
from lerobot.utils.feature_utils import combine_feature_dicts
from lerobot.utils.utils import log_say

FPS              = 30
EPISODE_TIME_SEC = 30   # seconds per demonstration
RESET_TIME_SEC   = 10   # pause between episodes for environment reset
TASK_DESCRIPTION = "Pick the red cube and place it in the bin"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--follower_port", default="/dev/ttyACM0")
    parser.add_argument("--repo_id",       required=True,  help="e.g. yourname/roarm_pickplace_v1")
    parser.add_argument("--num_episodes",  type=int, default=50)
    args = parser.parse_args()

    # Raspberry Pi Camera Module 3 (MIPI CSI) — continuous autofocus, macro range
    camera_config = {
        "wrist": Picamera2CameraConfig(
            width=640, height=480, fps=FPS,
            af_mode=2,   # continuous autofocus
            af_range=2,  # macro (gripper tip ~11.5cm from lens)
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
