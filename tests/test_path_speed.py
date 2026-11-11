import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'web'))
from path_speed import PathSpeedProfile


def test_straight_approach_and_new_goal_reset():
    profile = PathSpeedProfile([[0, 0], [1, 0], [2, 0]])
    assert profile.limit(0, 0, .1) == .1
    assert abs(profile.limit(1.8, 0, .1) - .05) < 1e-6
    assert profile.limit(1.96, 0, .1) == .05
    assert profile.limit(1.90, 0, .1) == .05
    assert PathSpeedProfile([[0, 0], [2, 0]]).limit(0, 0, .1) == .1


def test_bend_limits_and_straight_sideways_motion():
    profile = PathSpeedProfile([[0, 0], [1, 0], [1, 1]])
    assert profile.limit(0, 0, .1) == .1
    assert profile.limit(.9, 0, .1) == .05
    assert profile.limit(1, .3, .1) == .1
    assert PathSpeedProfile([[0, 0], [0, 1]]).limit(0, 0, .1) == .1


def test_duplicate_points_and_short_path():
    profile = PathSpeedProfile([[0, 0], [0, 0], [-.1, 0]])
    assert profile.limit(-.08, 0, .1) == .05
    assert PathSpeedProfile([[0, 0], [0, 0]]).limit(0, 0, .1) == .05
