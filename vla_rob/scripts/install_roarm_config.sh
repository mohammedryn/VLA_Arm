#!/bin/bash
# Copy RoArm M2-S custom files into the LeRobot installation and patch utils.py.
# Run once after cloning / pulling this branch on the RPi5 or GPU machine.
#
# Prerequisites:
#   ~/lerobot  — HuggingFace LeRobot repo already installed (pip install -e ".[feetech]")
#   This script runs from the VLA_Arm repo root.

set -e

LEROBOT="${LEROBOT_PATH:-$HOME/lerobot}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "Installing RoArm M2-S config files into $LEROBOT ..."

# 1. Copy robot package
cp -r "$REPO_ROOT/lerobot/src/lerobot/robots/roarm_m2s" \
       "$LEROBOT/src/lerobot/robots/"
echo "  ✓ robots/roarm_m2s/"

# 2. Copy custom Picamera2 driver
cp "$REPO_ROOT/lerobot/src/lerobot/cameras/picamera2_camera.py" \
   "$LEROBOT/src/lerobot/cameras/"
echo "  ✓ cameras/picamera2_camera.py"

# 3. Patch utils.py — add elif branch for roarm_m2s_follower
python3 "$SCRIPT_DIR/patch_lerobot_utils.py" "$LEROBOT/src/lerobot/robots/utils.py"
echo "  ✓ robots/utils.py patched"

echo ""
echo "Done. Verify with:"
echo "  python $SCRIPT_DIR/test_roarm_config.py"
