#!/usr/bin/env python3
"""
Deploy a trained ACT policy on the RoArm M2-S for autonomous pick-and-place.

Camera: Raspberry Pi Camera Module 3 (IMX708, MIPI CSI) via picamera2.

NOTE on RPi5 inference speed: RPi5 ARM CPU takes ~10-15s per ACT pass.
With 100-step action chunking you re-infer every ~3.3s — always blocking on
inference. Run this script on the GPU laptop with the servo adapter exposed
via usbip. See Ryan's Task 7 for usbip setup.

Usage (GPU laptop with usbip, or RPi5 directly):
    python scripts/rollout_roarm.py \\
        --model_id YOUR_HF_USERNAME/roarm_act_v1 \\
        --port /dev/ttyACM0 \\
        --duration 60

Press Ctrl+C to stop cleanly.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # repo root on sys.path

from firmware.tools.safety_listener import SafetyMonitor

from lerobot.cameras.picamera2_camera import Picamera2CameraConfig
from lerobot.configs import PreTrainedConfig
from lerobot.robots.roarm_m2s import RoArmM2SFollower, RoArmM2SFollowerConfig
from lerobot.rollout import BaseStrategyConfig, RolloutConfig, build_rollout_context
from lerobot.rollout.inference import SyncInferenceConfig
from lerobot.rollout.strategies import BaseStrategy
from lerobot.utils.process import ProcessSignalHandler
from lerobot.utils.utils import init_logging

FPS = 30


def main():
    init_logging()

    parser = argparse.ArgumentParser()
    parser.add_argument("--model_id", required=True, help="HF model ID or local checkpoint path")
    parser.add_argument("--port",     default="/dev/ttyACM0")
    parser.add_argument("--duration", type=int, default=60, help="Episode duration in seconds")
    args = parser.parse_args()

    camera_config = {
        "wrist": Picamera2CameraConfig(
            width=640, height=480, fps=FPS,
            af_mode=2,   # continuous autofocus
            af_range=2,  # macro
        )
    }

    robot_config = RoArmM2SFollowerConfig(
        port=args.port,
        id="roarm_main",
        cameras=camera_config,
        use_degrees=True,
    )

    policy_config = PreTrainedConfig.from_pretrained(args.model_id)
    policy_config.pretrained_path = args.model_id

    cfg = RolloutConfig(
        robot=robot_config,
        policy=policy_config,
        strategy=BaseStrategyConfig(),
        inference=SyncInferenceConfig(),
        fps=FPS,
        duration=args.duration,
        task="Pick the red cube and place it in the bin",
    )

    signal_handler = ProcessSignalHandler(use_threads=True)

    # Safety monitor on ESP32 at /dev/ttyUSB0 — triggers clean shutdown on hard contact
    monitor = SafetyMonitor(
        port="/dev/ttyUSB0",
        baud=2_000_000,
        on_estop=signal_handler.trigger_shutdown,
    )
    monitor.start()

    try:
        ctx = build_rollout_context(cfg, signal_handler.shutdown_event)
        strategy = BaseStrategy(cfg.strategy)
        try:
            strategy.setup(ctx)
            strategy.run(ctx)
        finally:
            strategy.teardown(ctx)
    finally:
        monitor.stop()


if __name__ == "__main__":
    main()
