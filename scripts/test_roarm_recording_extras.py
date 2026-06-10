import pytest

from scripts.roarm_recording_extras import (
    EPISODES_PER_TASK,
    POSITIONS,
    TASK_DESCRIPTIONS,
    episode_setup_message,
    position_for_episode,
    task_for_episode,
)


def test_task_for_episode_first_block_is_phrase_a():
    assert task_for_episode(0) == TASK_DESCRIPTIONS[0]
    assert task_for_episode(EPISODES_PER_TASK - 1) == TASK_DESCRIPTIONS[0]


def test_task_for_episode_second_block_is_phrase_b():
    assert task_for_episode(EPISODES_PER_TASK) == TASK_DESCRIPTIONS[1]
    assert task_for_episode(2 * EPISODES_PER_TASK - 1) == TASK_DESCRIPTIONS[1]


def test_task_for_episode_out_of_range_raises():
    with pytest.raises(ValueError, match="exceeds"):
        task_for_episode(2 * EPISODES_PER_TASK)


def test_position_for_episode_rotates_every_ten_within_block():
    assert position_for_episode(0) == POSITIONS[0]
    assert position_for_episode(9) == POSITIONS[0]
    assert position_for_episode(10) == POSITIONS[1]
    assert position_for_episode(39) == POSITIONS[3]


def test_position_for_episode_wraps_for_second_task_block():
    # episode 40 is the first episode of phrase B, position rotation restarts
    assert position_for_episode(40) == POSITIONS[0]


def test_episode_setup_message_flags_lighting_change_at_block_starts():
    msg0 = episode_setup_message(0)
    msg1 = episode_setup_message(1)
    assert "vary lighting" in msg0
    assert "vary lighting" not in msg1
    assert TASK_DESCRIPTIONS[0] in msg0
    assert POSITIONS[0] in msg0
