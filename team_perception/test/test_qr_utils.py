"""Unit tests for qr_utils. Run from the package folder: pytest test/test_qr_utils.py -v."""
import os

import cv2
import numpy as np
import pytest

from team_perception import qr_utils
from team_perception.qr_utils import decode_qr, depth_at, deproject, draw_detections

IMAGES = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_images')


def _qr_modules(text):
    """
    QR as a uint8 image (0 = black, 255 = white), no quiet zone.

    Older OpenCV (e.g. 4.5 from Ubuntu 22.04 apt) can only READ QR codes,
    so fall back to the `qrcode` package:  sudo apt install python3-qrcode
    """
    if hasattr(cv2, 'QRCodeEncoder'):
        return cv2.QRCodeEncoder.create().encode(text)
    import qrcode
    q = qrcode.QRCode(border=0)
    q.add_data(text)
    q.make(fit=True)
    return np.where(np.array(q.get_matrix(), dtype=bool), 0, 255).astype(np.uint8)


def make_qr(text, size=300, border=40):
    qr = _qr_modules(text)
    qr = cv2.resize(qr, (size, size), interpolation=cv2.INTER_NEAREST)
    img = np.full((size + 2 * border, size + 2 * border), 255, np.uint8)
    img[border:border + size, border:border + size] = qr
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def test_decodes_plain_code():
    found = decode_qr(make_qr('ROOM:KITCHEN'))
    assert [t for t, _ in found] == ['ROOM:KITCHEN']


def test_decodes_code_in_bigger_scene():
    scene = np.full((720, 1280, 3), 120, np.uint8)
    qr = make_qr('SURVIVOR-3', size=120, border=12)
    scene[400:400 + qr.shape[0], 900:900 + qr.shape[1]] = qr
    found = decode_qr(scene)
    assert found and found[0][0] == 'SURVIVOR-3'
    u, v = found[0][1].mean(axis=0)
    assert 900 < u < 900 + qr.shape[1] and 400 < v < 400 + qr.shape[0]


def test_tilted_code():
    img = make_qr('TILT')
    h, w = img.shape[:2]
    src = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    dst = np.float32([[40, 20], [w - 10, 0], [w, h], [0, h - 30]])
    warped = cv2.warpPerspective(img, cv2.getPerspectiveTransform(src, dst), (w, h),
                                 borderValue=(255, 255, 255))
    assert [t for t, _ in decode_qr(warped)] == ['TILT']


def test_no_code():
    assert decode_qr(np.full((480, 640, 3), 200, np.uint8)) == []


def test_depth_mm_and_m_ignores_zeros():
    d16 = np.zeros((10, 10), np.uint16)
    d16[4:7, 4:7] = 1500
    d16[5, 5] = 0
    assert abs(depth_at(d16, 5, 5) - 1.5) < 1e-6
    d32 = np.full((10, 10), np.nan, np.float32)
    d32[5, 5] = 2.0
    assert depth_at(d32, 5, 5) == 2.0
    assert depth_at(np.zeros((10, 10), np.uint16), 5, 5) is None


def test_deproject_centre_and_offset():
    assert deproject(320, 240, 2.0, 600, 600, 320, 240) == (0.0, 0.0, 2.0)
    x, y, z = deproject(620, 240, 2.0, 600, 600, 320, 240)
    assert abs(x - 1.0) < 1e-9 and y == 0.0


def test_fast_mode_skips_fallbacks_but_still_decodes():
    found = decode_qr(make_qr('FAST'), try_upscale=False, try_clahe=False)
    assert [t for t, _ in found] == ['FAST']


def test_draw_detections_labels_and_header():
    img = make_qr('LABEL-ME')
    found = decode_qr(img)
    out = draw_detections(img.copy(), found, {'LABEL-ME': ['map (1.00, 2.00, 0.00)']},
                          header=['status line', 'éàü non-ascii is fine'])
    assert out.shape == img.shape and (out != img).any()
    # no detections and an empty image must not crash either
    draw_detections(np.zeros((50, 60, 3), np.uint8), [], header=['x'])


def _photo(name):
    img = cv2.imread(os.path.join(IMAGES, name))
    assert img is not None, f'missing test image {name}'
    return img


@pytest.mark.skipif(qr_utils._zx is None, reason='needs zxing-cpp: pip install zxing-cpp==2.3.0')
def test_real_photo_tilted_code():
    # Hand-held printout seen in perspective; pyzbar alone cannot read this one.
    found = decode_qr(_photo('test_c.jpg'), try_upscale=False, try_clahe=False)
    assert [t for t, _ in found] == ['C']
    u, v = found[0][1].mean(axis=0)
    assert 1100 < u < 1700 and 300 < v < 1100   # centre of the code in the 2560x1440 photo


@pytest.mark.xfail(reason='paper is curved; no decoder reads it - mount codes on card',
                   strict=False)
def test_real_photo_curved_paper():
    assert [t for t, _ in decode_qr(_photo('test_D.jpg'))] == ['D']


@pytest.mark.skipif(qr_utils._zbar is None, reason='needs pyzbar')
def test_still_works_without_zxing(monkeypatch):
    monkeypatch.setattr(qr_utils, '_zx', None)
    assert [t for t, _ in decode_qr(make_qr('NO-ZXING'))] == ['NO-ZXING']


def test_flatten_decodes_given_quad():
    gray = cv2.cvtColor(make_qr('FLAT'), cv2.COLOR_BGR2GRAY)
    quad = np.float32([[40, 40], [340, 40], [340, 340], [40, 340]])
    found = qr_utils._decode_flattened(gray, quad)
    assert [t for t, _ in found] == ['FLAT'] and np.allclose(found[0][1], quad)
