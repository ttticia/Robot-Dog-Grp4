"""Tests for maze_explorer.gait_utils (foot lift from joint angles)."""

import math

from maze_explorer.gait_utils import foot_height, LEGS, StepHeightMeter


def test_foot_height_standing_pose_is_about_body_height():
    # Typical stance: hip -40 deg, knee 80 deg -> foot ~0.31 m below the hip.
    z = foot_height(0.0, math.radians(-40), math.radians(80))
    assert -0.33 < z < -0.29
    # /joint_states negates every angle: same height.
    assert abs(foot_height(0.0, math.radians(40), math.radians(-80)) - z) < 1e-12
    # Straight leg: full length.
    assert abs(foot_height(0.0, 0.0, 0.0) + 0.41) < 1e-12


def legs_sample(hip_y, knee):
    """Return (names, positions) with every leg at the same angles."""
    names, pos = [], []
    for jx, jy, jk in LEGS.values():
        names += [jx, jy, jk]
        pos += [0.0, hip_y, knee]
    return names, pos


def simulate(meter, lift_knee_deg, seconds=5.0, rate=100.0):
    """Feed a stepping gait: the knee bends extra by up to ``lift_knee_deg`` at 2 Hz."""
    for i in range(int(seconds * rate)):
        t = i / rate
        bend = math.radians(lift_knee_deg) * max(0.0, math.sin(2 * math.pi * 2.0 * t))
        meter.add(t, *legs_sample(math.radians(-40) - bend / 2, math.radians(80) + bend))


def test_meter_tells_high_steps_from_low_steps():
    low, high = StepHeightMeter(4.0), StepHeightMeter(4.0)
    simulate(low, 10)
    simulate(high, 35)
    assert 0.0 < low.lift() < high.lift()
    assert high.lift() > 2 * low.lift()
    assert set(high.lifts()) == set(LEGS)
    assert 3.5 < high.span_s() <= 4.0      # window keeps only the last 4 s


def test_meter_standing_still_reads_zero_and_empty_reads_none():
    m = StepHeightMeter()
    assert m.lift() is None and m.standing_height() is None
    for i in range(50):
        m.add(i / 100, *legs_sample(math.radians(-40), math.radians(80)))
    assert m.lift() < 1e-9
    assert -0.33 < m.standing_height() < -0.29
