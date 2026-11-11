"""End-to-end tests for the annotate tool. Run from the package folder: pytest test/ -v."""
import csv
import json
import os

import cv2
import numpy as np
import pytest

from team_perception.annotate import run


def _qr_modules(text):
    if hasattr(cv2, 'QRCodeEncoder'):
        return cv2.QRCodeEncoder.create().encode(text)
    import qrcode  # Ubuntu 22.04's OpenCV cannot encode: sudo apt install python3-qrcode
    q = qrcode.QRCode(border=0)
    q.add_data(text)
    q.make(fit=True)
    return np.where(np.array(q.get_matrix(), dtype=bool), 0, 255).astype(np.uint8)


def scene(qr_text=None, person=False, w=640, h=480, x=300, y=150, size=180):
    """Grey frame, optionally with a QR code and/or a 'person' marker (white top-left)."""
    img = np.full((h, w, 3), 100, np.uint8)
    if person:
        img[:32, :32] = 255
    if qr_text:
        qr = cv2.resize(_qr_modules(qr_text), (size, size), interpolation=cv2.INTER_NEAREST)
        img[y - 20:y + size + 20, x - 20:x + size + 20] = 255
        img[y:y + size, x:x + size] = cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR)
    return img


class FakeDetector:
    """Reports one person on frames with a white top-left corner (survives JPEG/MJPG)."""

    def detect(self, img):
        if img[:20, :20].mean() > 200:
            return [(np.array([10, 10, 200, 400], np.float32), 0.9)]
        return []


@pytest.fixture
def media(tmp_path):
    photos = tmp_path / 'photos'
    photos.mkdir()
    cv2.imwrite(str(photos / 'qr_only.png'), scene('ROOM:KITCHEN'))
    cv2.imwrite(str(photos / 'person_only.png'), scene(person=True))
    cv2.imwrite(str(photos / 'both.png'), scene('A', person=True))
    video = str(tmp_path / 'clip.avi')
    vw = cv2.VideoWriter(video, cv2.VideoWriter_fourcc(*'MJPG'), 10, (640, 480))
    for i in range(30):   # 1 s nothing, 1 s QR "A", 1 s person + QR "B"
        vw.write(scene(None if i < 10 else ('A' if i < 20 else 'B'), person=i >= 20))
    vw.release()
    return str(photos), video, str(tmp_path / 'runs')


def _summary(run_dir):
    res = json.load(open(os.path.join(run_dir, 'summary.json')))
    return res, {os.path.basename(r['input']): r for r in res['results']}


def _kinds(run_dir):
    with open(os.path.join(run_dir, 'detections.csv')) as f:
        return {r['kind'] for r in csv.DictReader(f)}


def test_requires_type(media):
    with pytest.raises(SystemExit):
        run([media[0]])


def test_qr_run(media):
    photos, video, root = media
    out = run(['--type', 'qr', photos, video, '--out-root', root, '--name', 'unit'])
    assert os.path.dirname(out) == os.path.join(root, 'qr')
    assert os.path.basename(out).endswith('_unit')
    assert sorted(os.listdir(os.path.join(out, 'input'))) == \
        ['both.png', 'clip.avi', 'person_only.png', 'qr_only.png']
    meta, res = _summary(out)
    assert meta['test_type'] == 'qr'
    assert res['qr_only.png']['qr_codes'] == ['ROOM:KITCHEN']
    assert res['person_only.png']['qr_codes'] == []
    assert 'people' not in res['both.png']
    assert [(c['text'], c['first_s']) for c in res['clip.avi']['qr_codes']] == \
        [('A', 1.0), ('B', 2.0)]
    cap = cv2.VideoCapture(os.path.join(out, 'annotated_clip.mp4'))
    assert int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) == 30
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc = ''.join(chr(fourcc >> 8 * k & 255) for k in range(4)).lower()
    if res['clip.avi']['codec'].startswith('avc1'):   # H.264 so VS Code/browsers can play it
        assert fourcc in ('avc1', 'h264')
    assert len(os.listdir(os.path.join(out, 'first_seen'))) == 2
    assert _kinds(out) == {'qr'}


def test_human_run(media):
    photos, video, root = media
    out = run(['-t', 'human', photos, video, '--out-root', root, '--no-copy'],
              detector=FakeDetector())
    assert os.path.dirname(out) == os.path.join(root, 'human')
    assert not os.path.exists(os.path.join(out, 'input'))
    _, res = _summary(out)
    assert res['person_only.png']['people'] == 1 and res['qr_only.png']['people'] == 0
    assert 'qr_codes' not in res['both.png']
    assert res['clip.avi']['person_frames_hit'] == 10
    assert res['clip.avi']['person_first_s'] == 2.0
    assert _kinds(out) == {'person'}


def test_both_run(media):
    photos, video, root = media
    out = run(['--type', 'both', photos, video, '--out-root', root, '--max-frames', '25'],
              detector=FakeDetector())
    assert os.path.dirname(out) == os.path.join(root, 'both')
    _, res = _summary(out)
    assert res['both.png']['qr_codes'] == ['A'] and res['both.png']['people'] == 1
    assert res['clip.avi']['frames'] == 25
    assert res['clip.avi']['person_frames_hit'] == 5
    assert {c['text'] for c in res['clip.avi']['qr_codes']} == {'A', 'B'}
    assert _kinds(out) == {'qr', 'person'}
    text = open(os.path.join(out, 'summary.txt')).read()
    assert 'type: both' in text and 'with a person:  2' in text and 'with a QR code: 2' in text


class BrokenDetector:
    def detect(self, img):
        raise RuntimeError('model exploded')


def test_status_file_done_and_no_partial_video(media, capsys):
    photos, video, root = media
    out = run(['-t', 'both', photos, video, '--out-root', root], detector=FakeDetector())
    status = open(os.path.join(out, 'STATUS.txt')).read()
    assert status.startswith('State:    DONE')
    assert '[done] copied 4 input(s)' in status
    assert '[done] 4/4 clip.avi: 30 frames' in status
    assert not [f for f in os.listdir(out) if '.partial' in f or f.endswith('.tmp')]
    printed = capsys.readouterr().out
    assert '30/30 frames (100.0%)' in printed and 'Status: DONE' in printed


def test_status_file_failed(media):
    photos, video, root = media
    with pytest.raises(RuntimeError):
        run(['-t', 'human', video, '--out-root', root], detector=BrokenDetector())
    out = os.path.join(root, 'human', os.listdir(os.path.join(root, 'human'))[0])
    status = open(os.path.join(out, 'STATUS.txt')).read()
    assert status.startswith('State:    FAILED') and 'model exploded' in status
