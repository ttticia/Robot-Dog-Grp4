"""Tests for qr_utils.DepthAlign: colour pixels into an unaligned depth image (Lite3 numbers)."""
import numpy as np
import pytest

from team_perception.poster_utils import REAL, edge_ring_test, scale_box
from team_perception.qr_utils import DepthAlign, DepthAlignState, depth_at

# The Lite3's Orbbec camera (robot_info.md section 3.1 and the teleop bag of 2026-11-12)
COLOUR_K = (461.92, 461.95, 319.89, 240.90)
DEPTH_K = (372.79, 372.79, 319.40, 241.50)
SIZE = (640, 480)
T = (0.01409, -0.00013, 0.00184)    # colour camera in the depth optical frame (from TF)
RNG = np.random.default_rng(1)


def test_assumed_aligned_is_plain_scaling():
    a = DepthAlign.assume_aligned(COLOUR_K, SIZE, (240, 320))
    assert a.pixel(320, 240) == pytest.approx((160, 120), abs=0.1)
    assert a.pixel(100, 50) == pytest.approx((50, 25), abs=0.01)
    assert a.box([100, 50, 200, 150]) == pytest.approx([50, 25, 100, 75], abs=0.01)


def test_lite3_edges_are_60_px_off_without_compensation():
    a = DepthAlign(COLOUR_K, SIZE, DEPTH_K)
    assert a.pixel(COLOUR_K[2], COLOUR_K[3]) == pytest.approx((DEPTH_K[2], DEPTH_K[3]))
    ud, vd = a.pixel(0, 0)
    assert ud == pytest.approx(61.2, abs=0.5)     # plain scaling would say 0
    assert vd == pytest.approx(47.1, abs=0.5)
    assert a.pixel(639, 479)[0] == pytest.approx(577.1, abs=0.5)


def test_parallax_shrinks_with_distance():
    a = DepthAlign(COLOUR_K, SIZE, DEPTH_K, T)
    u0 = a.pixel(320, 240)[0]
    shift_1m = a.pixel(320, 240, z=1.0)[0] - u0
    shift_3m = a.pixel(320, 240, z=3.0)[0] - u0
    assert shift_1m == pytest.approx(DEPTH_K[0] * T[0] / 1.0, rel=0.02)   # ~5.3 px
    assert shift_3m == pytest.approx(shift_1m / 3, rel=0.02)


def test_image_size_rescales_pixels():
    a = DepthAlign(COLOUR_K, SIZE, DEPTH_K)
    assert a.pixel(160, 120, image_size=(320, 240)) == pytest.approx(a.pixel(320, 240))


def render_depth(colour_box, person_z, wall_z=3.0, noise=0.005):
    """Depth image as the depth camera sees a person (flat box + bulge) before a wall."""
    a = DepthAlign(COLOUR_K, SIZE, DEPTH_K, T)
    d = np.full((SIZE[1], SIZE[0]), wall_z, np.float32)
    x1, y1, x2, y2 = (int(round(v)) for v in a.box(colour_box, z=person_z))
    cx, bw = (x1 + x2) / 2, x2 - x1
    xs = np.arange(x1, x2)
    bulge = 0.10 * np.clip(1 - np.abs(xs - cx) / (bw / 2), 0, 1)
    d[y1:y2, x1:x2] = person_z - bulge
    d += RNG.normal(0, noise, d.shape).astype(np.float32)
    return d, a


def test_depth_at_finds_a_narrow_object_near_the_edge():
    colour_box = [20, 150, 50, 350]               # 30 px wide, at the left edge
    d, a = render_depth(colour_box, 1.2)
    z, ud, vd = a.depth_at(d, 35, 250)
    assert z == pytest.approx(1.1, abs=0.1)
    assert depth_at(d, 35, 250) == pytest.approx(3.0, abs=0.05)   # unaligned lookup: the wall


def test_real_person_at_edge_needs_compensation():
    """Plain scaling partly looks at the wall beside the person and misses the jump."""
    colour_box = [10, 120, 90, 400]
    d, a = render_depth(colour_box, 1.5)
    naive, _ = edge_ring_test(d, scale_box(colour_box, SIZE, d.shape))
    assert naive != REAL                         # the bug this compensation fixes
    box0 = a.box(colour_box)
    box = a.box(colour_box, z=1.5)
    assert edge_ring_test(d, box)[0] == REAL
    assert abs(box[0] - box0[0]) < 5             # parallax is a small correction


def test_state_reasons_and_tf():
    s = DepthAlignState(depth_info=True)
    assert s.get((480, 640)) == (None, 'no colour camera_info yet')
    s.set_colour(COLOUR_K, SIZE, 'camera_color_optical_frame')
    assert s.get((480, 640)) == (None, 'no depth camera_info yet')
    s.set_depth(DEPTH_K, 'camera_depth_optical_frame')
    assert s.needs_tf() == ('camera_depth_optical_frame', 'camera_color_optical_frame')
    a, why = s.get((480, 640))
    assert a.t == (0, 0, 0) and 't = 0' in why
    s.t = T
    a, why = s.get((480, 640))
    assert why == 'ok' and a.t == T and s.needs_tf() is None


def test_state_same_frame_and_assumed_aligned():
    s = DepthAlignState(depth_info=True)
    s.set_colour(COLOUR_K, SIZE, 'cam')
    s.set_depth(COLOUR_K, 'cam')                 # an aligned topic: same frame, no TF needed
    assert s.needs_tf() is None and s.t == (0.0, 0.0, 0.0)
    s = DepthAlignState(depth_info=False)
    s.set_colour(COLOUR_K, SIZE, 'cam')
    a, why = s.get((480, 640))
    assert why == 'assumed aligned' and a.pixel(100, 100) == pytest.approx((100, 100))
