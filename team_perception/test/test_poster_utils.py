"""Tests for poster_utils on synthetic depth scenes and maps."""
import numpy as np
import pytest

from team_perception.poster_utils import (POSTER, REAL, UNKNOWN, PosterTracker, RingParams,
                                          StaticMap, combine, edge_ring_test, scale_box,
                                          static_map_test, to_metres, torso_pixel_and_depth)

W, H = 640, 480
BOX = [220, 100, 420, 420]          # a person-sized box in the middle of the image
RNG = np.random.default_rng(0)


def plane(z_left, z_right, noise=0.005):
    """
    Make depth (m) of a flat surface whose depth goes from z_left to z_right.

    Uses 1/z linear in u, which is exactly what a tilted plane looks like to a camera.
    """
    u = np.arange(W, dtype=np.float64)
    inv = 1 / z_left + (1 / z_right - 1 / z_left) * u / (W - 1)
    d = np.tile(1 / inv, (H, 1))
    return (d + RNG.normal(0, noise, d.shape)).astype(np.float32)


def person_in_front(background_z, person_z, box=BOX, noise=0.005):
    """Make a real person (head + torso + legs shape) standing in front of a background."""
    d = plane(background_z, background_z, noise)
    x1, y1, x2, y2 = box
    bw, bh = x2 - x1, y2 - y1
    body = np.zeros((H, W), bool)
    cx = (x1 + x2) // 2
    body[y1:y1 + bh // 6, cx - bw // 8:cx + bw // 8] = True                  # head
    body[y1 + bh // 6:y1 + 2 * bh // 3, x1 + bw // 10:x2 - bw // 10] = True  # torso + arms
    body[y1 + 2 * bh // 3:y2, cx - bw // 4:cx + bw // 4] = True              # legs
    # a body is not flat: centre of the torso ~10 cm nearer than its sides
    xs = np.arange(W)
    bulge = 0.10 * np.clip(1 - np.abs(xs - cx) / (bw / 2), 0, 1)
    d[body] = (person_z - np.tile(bulge, (H, 1)))[body]
    return d


def test_flat_poster_on_wall():
    label, m = edge_ring_test(plane(2.0, 2.0), BOX)
    assert label == POSTER, m
    assert m['sides_used'] == 3


def test_tilted_poster_on_wall():
    label, m = edge_ring_test(plane(1.5, 3.0), BOX)       # wall seen at a steep angle
    assert label == POSTER, m


def test_real_person_in_front_of_wall():
    label, m = edge_ring_test(person_in_front(4.0, 2.0), BOX)
    assert label == REAL, m


def test_person_close_to_wall_is_not_called_poster():
    # 15 cm in front of the wall: not a clear jump, but not the same surface either
    label, m = edge_ring_test(person_in_front(2.15, 2.0), BOX)
    assert label == UNKNOWN, m


def test_no_depth_and_too_far_are_unknown():
    assert edge_ring_test(np.full((H, W), np.nan, np.float32), BOX)[0] == UNKNOWN
    assert edge_ring_test(plane(5.0, 5.0), BOX)[0] == UNKNOWN


def test_box_at_image_edge_uses_remaining_sides():
    label, m = edge_ring_test(plane(2.0, 2.0), [0, 0, 200, 300])
    assert m['sides']['left'] == 'outside image' and m['sides']['top'] == 'outside image'
    assert label == UNKNOWN            # only one usable side (right) < min_sides
    label, _ = edge_ring_test(plane(2.0, 2.0), [0, 0, 200, 300], RingParams(min_sides=1))
    assert label == POSTER


def test_depth_units_and_scaling():
    mm = np.full((4, 4), 2000, np.uint16)
    mm[0, 0] = 0
    d = to_metres(mm)
    assert d[1, 1] == pytest.approx(2.0) and np.isnan(d[0, 0])
    assert scale_box([100, 50, 200, 150], [1280, 720], (360, 640)) == [50, 25, 100, 75]
    u, v, z = torso_pixel_and_depth(plane(2.0, 2.0), BOX)
    assert z == pytest.approx(2.0, abs=0.01) and BOX[0] < u < BOX[2]


# --- map --------------------------------------------------------------------------------
def make_map():
    """10 x 10 m map at 5 cm, origin (-5, -5): free, one wall along x = 2 m, unknown y > 4."""
    res = 0.05
    n = 200
    g = np.zeros((n, n), np.int16)
    g[:, int((2.0 + 5) / res)] = 100          # wall at x = 2.0
    g[int((4.0 + 5) / res):, :] = -1          # unexplored strip
    return StaticMap(g.ravel(), n, n, res, (-5.0, -5.0))


def test_map_on_wall_free_and_unknown():
    m = make_map()
    assert static_map_test(m, 1.95, 0.0)[0] == POSTER     # 5 cm in front of the wall
    assert static_map_test(m, 0.0, 0.0)[0] == REAL        # open floor
    assert static_map_test(m, 1.55, 0.0)[0] == UNKNOWN    # 45 cm from the wall
    assert static_map_test(m, 0.0, 4.5)[0] == UNKNOWN     # unexplored
    assert static_map_test(m, 50.0, 0.0)[0] == UNKNOWN    # off the map
    assert static_map_test(None, 0.0, 0.0)[0] == UNKNOWN


@pytest.mark.parametrize('ring,map_label,mode,expected', [
    (REAL, POSTER, 'veto', REAL),
    (POSTER, POSTER, 'require', POSTER),
    (POSTER, UNKNOWN, 'require', UNKNOWN),
    (POSTER, UNKNOWN, 'veto', POSTER),
    (POSTER, REAL, 'veto', UNKNOWN),
    (POSTER, REAL, 'off', POSTER),
    (UNKNOWN, POSTER, 'veto', UNKNOWN),
    (UNKNOWN, REAL, 'veto', REAL),
])
def test_combine(ring, map_label, mode, expected):
    assert combine(ring, map_label, mode) == expected


def test_tracker_votes_and_reports_once():
    t = PosterTracker(merge_radius_m=0.5, window=8, need=5, repeat_after_s=60)
    reports = []
    labels = [POSTER, UNKNOWN, POSTER, POSTER, REAL, POSTER, POSTER, POSTER]
    for i, lab in enumerate(labels):
        track, rep = t.update((1.0 + 0.02 * i, 2.0, 0.5), lab, now=i)
        reports.append(rep)
    assert reports.index(True) == 6          # 5th poster vote arrives on the 7th frame
    assert sum(reports) == 1                 # reported once
    assert len(t.tracks) == 1 and t.votes(track) == '6/8'
    other, rep = t.update((5.0, 2.0, 0.5), POSTER, now=9)
    assert other.id != track.id and not rep
