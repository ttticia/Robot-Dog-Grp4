"""
Person detection with Ultralytics YOLO, with no ROS imports so it can be tested anywhere.

YOLO's COCO class 0 ('person') fires on real people AND on pictures of people
(posters, printouts, screens), which is what the challenge's human targets are.

The model weights (default yolo11n.pt, ~5 MB) are downloaded on first use into
~/.cache/team_perception/ instead of the current folder.
"""
import os

import numpy as np

PERSON = 0
MODEL_DIR = os.path.expanduser('~/.cache/team_perception')


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


def box_depth_pixel(box):
    """Pixel to sample depth for a person box: centre, a bit above middle (torso)."""
    x1, y1, x2, y2 = box
    return (x1 + x2) / 2.0, y1 + 0.4 * (y2 - y1)
