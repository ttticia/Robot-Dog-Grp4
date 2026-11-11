"""
Person detection with Ultralytics YOLO, with no ROS imports so it can be tested anywhere.

YOLO's COCO class 0 ('person') fires on real people AND on pictures of people
(posters, printouts, screens), which is what the challenge's human targets are.

The model weights (default yolo11n.pt, ~5 MB) are downloaded on first use into
~/.cache/team_perception/ instead of the current folder.
"""
import os

import numpy as np

from team_perception.qr_utils import _text_block

PERSON = 0
MODEL_DIR = os.path.expanduser('~/.cache/team_perception')
BOX_COLOUR = (0, 140, 255)      # orange (BGR); QR boxes are green
LABEL_BG = (0, 90, 180)


def resolve_model(model):
    """Bare names like 'yolo11n.pt' live in MODEL_DIR; paths are used as given."""
    if os.sep in model or os.path.exists(model):
        return os.path.expanduser(model)
    os.makedirs(MODEL_DIR, exist_ok=True)
    return os.path.join(MODEL_DIR, model)


class HumanDetector:
    """Wraps a YOLO model and returns person boxes only."""

    def __init__(self, model='yolo11n.pt', conf=0.4, imgsz=640, device='cpu'):
        from ultralytics import YOLO   # imported here: slow (~1 s) and pulls in torch
        self.model = YOLO(resolve_model(model))
        self.conf = conf
        self.imgsz = imgsz
        self.device = device

    def detect(self, bgr):
        """Return [(box[x1, y1, x2, y2] float32, confidence)] for people in a BGR image."""
        r = self.model.predict(bgr, classes=[PERSON], conf=self.conf, imgsz=self.imgsz,
                               device=self.device, verbose=False)[0]
        boxes = r.boxes.xyxy.cpu().numpy().astype(np.float32)
        confs = r.boxes.conf.cpu().numpy()
        return [(b, float(c)) for b, c in zip(boxes, confs)]


def draw_humans(img, detections, labels=None, header=None, reserve_lines=0):
    """
    Draw person boxes (orange) with confidence on img in place and return it.

    labels:        optional list of extra label lines per detection (same order).
    header:        optional status lines drawn top-left.
    reserve_lines: keep labels below a status panel of this many lines that someone else
                   draws afterwards (e.g. draw_detections' header in 'both' mode).
    """
    import cv2
    scale = max(0.5, img.shape[0] / 1000.0)
    n_header = len(header) if header else reserve_lines
    top = int((n_header * 28 + 10) * scale * 0.8) if n_header else 0
    for i, (box, conf) in enumerate(detections):
        x1, y1, x2, y2 = (int(v) for v in box)
        cv2.rectangle(img, (x1, y1), (x2, y2), BOX_COLOUR, max(2, int(3 * scale)))
        lines = [f'person {conf:.2f}'] + (list(labels[i]) if labels else [])
        if y1 - top > 30 * scale * len(lines):
            _text_block(img, lines, x1, y1 - 2, scale, (255, 255, 255), LABEL_BG, True)
        else:
            _text_block(img, lines, x1, max(y1, top) + 2, scale, (255, 255, 255), LABEL_BG)
    if header:
        _text_block(img, header, 0, 0, scale * 0.8, (255, 255, 255), (40, 40, 40))
    return img


def box_depth_pixel(box):
    """Pixel to sample depth for a person box: centre, a bit above middle (torso)."""
    x1, y1, x2, y2 = box
    return (x1 + x2) / 2.0, y1 + 0.4 * (y2 - y1)
