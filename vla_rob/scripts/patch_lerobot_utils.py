"""
Patch lerobot/src/lerobot/robots/utils.py to register roarm_m2s_follower.

Adds the elif branch immediately after the so101_follower block.
Safe to run multiple times — skips if the patch is already present.

Usage:
    python patch_lerobot_utils.py /path/to/lerobot/src/lerobot/robots/utils.py
"""

import sys

NEEDLE = '    elif config.type == "so101_follower":'
INSERTION = """\
    elif config.type == "roarm_m2s_follower":
        from .roarm_m2s import RoArmM2SFollower

        return RoArmM2SFollower(config)
"""


def patch(path: str) -> None:
    with open(path, "r") as f:
        src = f.read()

    if 'roarm_m2s_follower' in src:
        print(f"Already patched: {path}")
        return

    if NEEDLE not in src:
        print(f"ERROR: anchor line not found in {path}")
        print(f"  Expected: {NEEDLE!r}")
        print("  Check that your LeRobot version matches the expected source.")
        sys.exit(1)

    # Find the end of the so101_follower block (two lines after NEEDLE)
    lines = src.splitlines(keepends=True)
    for i, line in enumerate(lines):
        if line.rstrip() == NEEDLE.rstrip():
            # Insert our block after the return statement of so101_follower
            # Lines: i   = elif config.type == "so101_follower":
            #        i+1 = '        from .so_follower import SO101Follower'
            #        i+2 = ''
            #        i+3 = '        return SO101Follower(config)'
            insert_at = i + 4
            lines.insert(insert_at, INSERTION)
            break

    with open(path, "w") as f:
        f.writelines(lines)

    print(f"Patched: {path}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <path/to/utils.py>")
        sys.exit(1)
    patch(sys.argv[1])
