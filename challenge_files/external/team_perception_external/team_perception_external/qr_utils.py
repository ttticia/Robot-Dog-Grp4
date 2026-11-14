"""
QR helper functions with no ROS imports, so they can be unit-tested anywhere.

Decoding backends, tried in this order on every image:
  1. zxing-cpp (pip install zxing-cpp==2.3.0): fast (~3 ms at 720p) and robust to
     perspective, rotation and uneven light. Optional but strongly recommended.
  2. pyzbar (apt: python3-pyzbar): catches some steep angles zxing misses (~50 ms).
     REQUIRED on Ubuntu 22.04 / ROS 2 Humble if zxing-cpp is absent: its OpenCV 4.5.4
     is built without QUIRC, so cv2.QRCodeDetector can locate codes but not decode them.
  3. If zxing located a code but could not read it, the code is flattened using the
     corners zxing found and both decoders try again (fixes tilted codes zbar misses).
  cv2.QRCodeDetector is used only if neither zxing-cpp nor pyzbar is installed.
"""
import numpy as np
import cv2

try:
    import zxingcpp as _zx
except ImportError:
    _zx = None

try:
    from pyzbar import pyzbar as _zbar
except ImportError:  # pyzbar or libzbar0 missing
    _zbar = None

_detector = cv2.QRCodeDetector()

BACKENDS = ' + '.join(n for n, m in (('zxing-cpp', _zx), ('pyzbar', _zbar)) if m) or 'OpenCV'


def _zbar_corners(sym):
    pts = np.asarray([(p.x, p.y) for p in sym.polygon], dtype=np.float32)
    if len(pts) != 4:  # zbar may return >4 hull points; reduce to a box
        pts = cv2.boxPoints(cv2.minAreaRect(pts)).astype(np.float32)
    return pts.reshape(4, 2)


def _zxing_decode(gray):
    """Return (decoded [(text, corners)], corners of codes located but not decodable)."""
    found, failed = [], []
    for r in _zx.read_barcodes(gray, formats=_zx.BarcodeFormat.QRCode, return_errors=True):
        p = r.position
        corners = np.float32([[p.top_left.x, p.top_left.y], [p.top_right.x, p.top_right.y],
                              [p.bottom_right.x, p.bottom_right.y],
                              [p.bottom_left.x, p.bottom_left.y]])
        if r.valid and r.text:
            found.append((r.text, corners))
        elif cv2.contourArea(corners) > 100:
            failed.append(corners)
    return found, failed


def _zbar_decode(gray):
    out = []
    for sym in _zbar.decode(gray, symbols=[_zbar.ZBarSymbol.QRCODE]):
        text = sym.data.decode('utf-8', errors='replace')
        if text:
            out.append((text, _zbar_corners(sym)))
    return out


def _opencv_decode(gray):
    results = []
    try:
        ok, texts, points, _ = _detector.detectAndDecodeMulti(gray)
    except cv2.error:
        ok, texts, points = False, [], None
    if ok and points is not None:
        for text, pts in zip(texts, points):
            if text:
                results.append((text, np.asarray(pts, dtype=np.float32).reshape(4, 2)))
    if not results:
        # Single-code decoder sometimes succeeds where Multi does not.
        try:
            text, pts, _ = _detector.detectAndDecode(gray)
        except cv2.error:
            text, pts = '', None
        if text and pts is not None:
            results.append((text, np.asarray(pts, dtype=np.float32).reshape(4, 2)))
    return results


def _decode_flattened(gray, corners, side=400, margin=80):
    """Warp the quad `corners` to a flat square and decode it; return [(text, corners)]."""
    dst = np.float32([[margin, margin], [margin + side, margin],
                      [margin + side, margin + side], [margin, margin + side]])
    size = side + 2 * margin
    flat = cv2.warpPerspective(gray, cv2.getPerspectiveTransform(corners, dst), (size, size),
                               flags=cv2.INTER_LINEAR, borderValue=255)
    texts = []
    if _zx is not None:
        texts = [t for t, _ in _zxing_decode(flat)[0]]
    if not texts and _zbar is not None:
        texts = [t for t, _ in _zbar_decode(flat)]
    return [(t, corners) for t in texts]


def _decode_once(gray):
    """Return a list of (text, corners[4x2]) found in one grayscale image."""
    gray = np.ascontiguousarray(gray)
    failed = []
    if _zx is not None:
        found, failed = _zxing_decode(gray)
        if found:
            return found
    if _zbar is not None:
        found = _zbar_decode(gray)
        if found:
            return found
    elif _zx is None:
        return _opencv_decode(gray)
    for corners in failed[:3]:
        found = _decode_flattened(gray, corners)
        if found:
            return found
    return []


def decode_qr(bgr, try_upscale=True, try_clahe=True):
    """
    Find and decode QR codes in a BGR (or grayscale) image.

    Tries the image as is, then a contrast-boosted version, then a 2x upscale
    (helps with small or distant codes). Returns [(text, corners)], corners in
    pixel coordinates of the ORIGINAL image.

    The fallbacks only run when nothing was found and cost ~5x a single pass, so
    for live video pass try_clahe=False, try_upscale=False on most frames.
    """
    gray = bgr if bgr.ndim == 2 else cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    found = _decode_once(gray)
    if found:
        return found

    if try_clahe:
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        found = _decode_once(clahe.apply(gray))
        if found:
            return found

    if try_upscale:
        big = cv2.resize(gray, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
        return [(t, c / 2.0) for t, c in _decode_once(big)]
    return []


def _ascii(s):
    # cv2.putText only draws ASCII; show anything else as '?'.
    return ''.join(ch if 32 <= ord(ch) < 127 else '?' for ch in s)


def _wrap(text, width, max_lines):
    lines = [text[i:i + width] for i in range(0, len(text), width)] or ['']
    if len(lines) > max_lines:
        lines = lines[:max_lines]
        lines[-1] = lines[-1][:-3] + '...'
    return lines


def _text_block(img, lines, x, y, scale, fg, bg, anchor_bottom=False):
    """Draw lines of text on a filled box with top-left (x, y), or bottom-left if anchored."""
    font, thick, pad = cv2.FONT_HERSHEY_SIMPLEX, max(1, int(round(scale * 2))), int(6 * scale)
    sizes = [cv2.getTextSize(t, font, scale, thick)[0] for t in lines]
    line_h = max(s[1] for s in sizes) + pad
    w = max(s[0] for s in sizes) + 2 * pad
    h = line_h * len(lines) + pad
    H, W = img.shape[:2]
    if anchor_bottom:
        y -= h
    x = int(min(max(0, x), max(0, W - w)))
    y = int(min(max(0, y), max(0, H - h)))
    cv2.rectangle(img, (x, y), (x + w, y + h), bg, -1)
    for i, t in enumerate(lines):
        cv2.putText(img, t, (x + pad, y + (i + 1) * line_h), font, scale, fg, thick,
                    cv2.LINE_AA)


def draw_detections(img, detections, info=None, header=None):
    """
    Draw QR boxes plus a label with the decoded text on img (in place) and return it.

    detections: [(text, corners[4x2])] as returned by decode_qr.
    info:       optional {text: [extra label lines]}, e.g. position or time seen.
    header:     optional list of lines for a status panel in the top-left corner.
    """
    info = info or {}
    scale = max(0.5, img.shape[0] / 1000.0)
    for text, corners in detections:
        pts = corners.astype(np.int32)
        cv2.polylines(img, [pts], True, (0, 255, 0), max(2, int(3 * scale)))
        cv2.circle(img, tuple(int(v) for v in pts[0]), int(5 * scale), (0, 0, 255), -1)
        lines = _wrap(_ascii(text), 40, 3) + [_ascii(s) for s in info.get(text, [])]
        x0, y0 = pts[:, 0].min(), pts[:, 1].min()
        # Label above the code (bottom of label touching the box); below it if no room.
        if y0 > 40 * scale * len(lines):
            _text_block(img, lines, x0, y0 - 4, scale, (255, 255, 255), (0, 110, 0),
                        anchor_bottom=True)
        else:
            _text_block(img, lines, x0, pts[:, 1].max() + 4, scale, (255, 255, 255),
                        (0, 110, 0))
    if header:
        _text_block(img, [_ascii(s) for s in header], 0, 0, scale * 0.8, (255, 255, 255),
                    (40, 40, 40))
    return img


def depth_at(depth, u, v, window=5):
    """
    Median depth in metres around pixel (u, v), ignoring zeros/NaNs.

    Accepts 16UC1 (millimetres) or 32FC1 (metres). Returns None if no valid pixel.
    """
    h, w = depth.shape[:2]
    u, v = int(round(u)), int(round(v))
    r = window // 2
    patch = depth[max(0, v - r):min(h, v + r + 1), max(0, u - r):min(w, u + r + 1)]
    patch = patch.astype(np.float64)
    if depth.dtype == np.uint16:
        patch = patch / 1000.0
    patch = patch[np.isfinite(patch) & (patch > 0.05)]
    if patch.size == 0:
        return None
    return float(np.median(patch))


def deproject(u, v, z, fx, fy, cx, cy):
    """Pixel + depth -> 3D point (x right, y down, z forward) in the camera optical frame."""
    return ((u - cx) * z / fx, (v - cy) * z / fy, z)


# --- unaligned depth ---------------------------------------------------------------------
class DepthAlign:
    """
    Map colour-image pixels into a depth image that is NOT aligned to the colour image.

    TEMPORARY stand-in for depth-to-colour registration in the camera driver (the Lite3's
    shared Orbbec driver publishes unaligned depth: fx 373 vs colour fx 462, sensors 14 mm
    apart). Both cameras are treated as pinhole cameras with parallel axes (the rotation
    between them is ignored; colour lens distortion too):
      colour_k, depth_k: (fx, fy, cx, cy) of each camera, from their camera_info
      colour_size:       (width, height) the colour intrinsics belong to
      t:                 position of the colour camera in the depth optical frame (m),
                         from TF; (0, 0, 0) if unknown (then up to ~5 px off at 1 m)
    With aligned depth (depth_k = colour_k scaled, t = 0) it is plain resolution scaling.
    """

    def __init__(self, colour_k, colour_size, depth_k, t=(0.0, 0.0, 0.0)):
        self.ck = tuple(float(v) for v in colour_k)
        self.size = colour_size
        self.depth_k = tuple(float(v) for v in depth_k)
        self.t = tuple(float(v) for v in t)

    @classmethod
    def assume_aligned(cls, colour_k, colour_size, depth_shape):
        """Depth registered to colour: only the resolution may differ."""
        sx = depth_shape[1] / float(colour_size[0])
        sy = depth_shape[0] / float(colour_size[1])
        fx, fy, cx, cy = colour_k
        return cls(colour_k, colour_size, (fx * sx, fy * sy, cx * sx, cy * sy))

    def pixel(self, u, v, z=None, image_size=None):
        """
        Colour pixel -> depth-image pixel.

        z (m, depth of the point) also corrects the parallax between the two sensors.
        image_size: (w, h) of the image (u, v) come from, if not colour_size.
        """
        if image_size and self.size and tuple(image_size) != tuple(self.size):
            u = u * self.size[0] / float(image_size[0])
            v = v * self.size[1] / float(image_size[1])
        fx, fy, cx, cy = self.ck
        x, y = (u - cx) / fx, (v - cy) / fy
        if z is not None and any(self.t):
            zz = max(z + self.t[2], 1e-3)
            x, y = (x * z + self.t[0]) / zz, (y * z + self.t[1]) / zz
        dfx, dfy, dcx, dcy = self.depth_k
        return dcx + dfx * x, dcy + dfy * y

    def box(self, box, z=None, image_size=None):
        """Map a colour box [x1, y1, x2, y2] into the depth image (parallax-corrected with z)."""
        x1, y1 = self.pixel(box[0], box[1], z, image_size)
        x2, y2 = self.pixel(box[2], box[3], z, image_size)
        return [x1, y1, x2, y2]

    def depth_at(self, depth, u, v, image_size=None, window=5):
        """
        Depth (m) of a colour pixel as (z, u_depth, v_depth), or None.

        Looks up once without parallax, then again at the parallax-corrected pixel.
        """
        ud, vd = self.pixel(u, v, None, image_size)
        z = depth_at(depth, ud, vd, window)
        if z is None:
            return None
        if any(self.t):
            ud2, vd2 = self.pixel(u, v, z, image_size)
            z2 = depth_at(depth, ud2, vd2, window)
            if z2 is not None:
                z, ud, vd = z2, ud2, vd2
        return z, ud, vd


class DepthAlignState:
    """
    Collects what DepthAlign needs from the camera_info topics and TF (ROS-free part).

    depth_info: True when a depth camera_info topic is configured (unaligned depth is
    compensated); False to assume the depth image is already aligned to colour.
    """

    def __init__(self, depth_info=True):
        self.depth_info = depth_info
        self.colour = None          # (k, size, frame)
        self.depth = None           # (k, frame)
        self.t = None               # colour camera position in the depth optical frame

    def set_colour(self, k, size, frame):
        self.colour = (tuple(k), tuple(size), frame)

    def set_depth(self, k, frame):
        if self.depth is None or self.depth[1] != frame:
            self.t = None
        self.depth = (tuple(k), frame)

    def needs_tf(self):
        """(depth_frame, colour_frame) if the TF between the sensors is still missing."""
        if not self.depth_info or self.colour is None or self.depth is None:
            return None
        if self.t is not None:
            return None
        if self.colour[2] == self.depth[1]:
            self.t = (0.0, 0.0, 0.0)
            return None
        return self.depth[1], self.colour[2]

    def get(self, depth_shape):
        """Return (DepthAlign or None, reason)."""
        if self.colour is None:
            return None, 'no colour camera_info yet'
        k, size, _ = self.colour
        if not self.depth_info:
            return DepthAlign.assume_aligned(k, size, depth_shape), 'assumed aligned'
        if self.depth is None:
            return None, 'no depth camera_info yet'
        if self.t is None:
            return DepthAlign(k, size, self.depth[0]), 'no TF between the sensors (t = 0)'
        return DepthAlign(k, size, self.depth[0], self.t), 'ok'
