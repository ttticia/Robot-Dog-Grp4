"""Tests for human_utils. Run from the package folder: pytest test/test_human_utils.py -v."""
import os

import cv2
import numpy as np
import pytest

from team_perception.human_utils import box_depth_pixel, draw_humans, resolve_model


def test_draw_humans_marks_box():
    img = np.full((480, 640, 3), 100, np.uint8)
    draw_humans(img, [(np.array([100, 50, 300, 400], np.float32), 0.87)], [['near']],
                ['status'])
    assert (img != 100).any()


def test_box_depth_pixel_is_torso():
    assert box_depth_pixel([0, 0, 100, 200]) == (50.0, 80.0)


def test_resolve_model_keeps_weights_out_of_cwd():
    assert resolve_model('yolo11n.pt').startswith(os.path.expanduser('~/.cache/'))
    assert resolve_model('/tmp/x/my.pt') == '/tmp/x/my.pt'


def test_real_yolo_finds_people_in_sample():
    """Needs ultralytics and the yolo11n.pt weights (downloaded on first run)."""
    ultralytics = pytest.importorskip('ultralytics')
    from team_perception.human_utils import HumanDetector
    bus = os.path.join(os.path.dirname(ultralytics.__file__), 'assets', 'bus.jpg')
    try:
        det = HumanDetector()
    except Exception as e:  # offline and no cached weights
        pytest.skip(f'could not load model: {e}')
    people = det.detect(cv2.imread(bus))
    assert len(people) >= 3          # bus.jpg shows 4 pedestrians
    assert all(c >= 0.4 for _, c in people)
