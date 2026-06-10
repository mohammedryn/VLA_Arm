"""Helper functions for language-phrase and scene-variation rotation in record_roarm.py."""

TASK_DESCRIPTIONS = [
    "pick the red cube and place it in the bin",
    "pick the blue cube and place it in the bin",
]
EPISODES_PER_TASK = 40

POSITIONS = ["front-left", "front-right", "back-left", "back-right"]
EPISODES_PER_POSITION = 10


def task_for_episode(episode_idx: int) -> str:
    """Return the single_task language phrase for a 0-indexed episode number."""
    task_idx = episode_idx // EPISODES_PER_TASK
    if task_idx >= len(TASK_DESCRIPTIONS):
        raise ValueError(
            f"episode_idx {episode_idx} exceeds planned "
            f"{len(TASK_DESCRIPTIONS) * EPISODES_PER_TASK} episodes"
        )
    return TASK_DESCRIPTIONS[task_idx]


def position_for_episode(episode_idx: int) -> str:
    """Return the staging position for a 0-indexed episode number.

    Position rotates through all 4 positions every EPISODES_PER_TASK episodes,
    restarting at the beginning of each task-phrase block.
    """
    block_idx = episode_idx % EPISODES_PER_TASK
    return POSITIONS[block_idx // EPISODES_PER_POSITION]


def episode_setup_message(episode_idx: int) -> str:
    """Operator-facing message printed before recording each episode."""
    task = task_for_episode(episode_idx)
    position = position_for_episode(episode_idx)
    msg = f'Episode {episode_idx + 1}: "{task}" -- stage object at {position}'
    if episode_idx % EPISODES_PER_POSITION == 0:
        msg += " -- vary lighting/background now"
    return msg
