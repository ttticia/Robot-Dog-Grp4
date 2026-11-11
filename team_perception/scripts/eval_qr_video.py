"""
Measure how well each QR decoding strategy works on a recorded clip (no ROS needed).

Input: a folder of frames (from webcam_sender_windows.py --record), a video file
(.mp4/.avi from any camera), or a ROS 2 bag is NOT supported - export frames first.

    python3 scripts/eval_qr_video.py /mnt/c/qr_clip
    python3 scripts/eval_qr_video.py clip.mp4 --save-misses misses/ --every 2

Prints, for each strategy, the share of frames it decoded and its speed. Frames
where at least one strategy found a code but the current one did not, and frames
where nothing decoded, are saved to --save-misses for inspection.
"""
import argparse
import glob
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from team_perception import qr_utils  # noqa: E402
from team_perception.qr_utils import decode_qr  # noqa: E402


def frames(path, every):
    if not os.path.exists(path):
        raise SystemExit(f'No such file or folder: {path}')
    if os.path.isdir(path):
        files = sorted(glob.glob(os.path.join(path, '*.jpg')) +
                       glob.glob(os.path.join(path, '*.png')))
        for i, f in enumerate(files):
            if i % every == 0:
                img = cv2.imread(f)
                if img is not None:
                    yield os.path.basename(f), img
    else:
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            raise SystemExit(f'Cannot open {path}')
        i = 0
        while True:
            ok, img = cap.read()
            if not ok:
                break
            if i % every == 0:
                yield f'frame_{i:05d}', img
            i += 1


def gray(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def strategies():
    s = {
        'current fast pass': lambda img: [t for t, _ in decode_qr(img, False, False)],
        'current full scan': lambda img: [t for t, _ in decode_qr(img)],
    }
    # Each backend on its own, to see which one is doing the work.
    if qr_utils._zx is not None:
        s['zxing-cpp only'] = lambda img: [t for t, _ in qr_utils._zxing_decode(gray(img))[0]]
    if qr_utils._zbar is not None:
        s['pyzbar only'] = lambda img: [t for t, _ in qr_utils._zbar_decode(gray(img))]
    return s


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('path', help='folder of .jpg/.png frames, or a video file')
    ap.add_argument('--every', type=int, default=1, help='use every Nth frame')
    ap.add_argument('--save-misses', metavar='DIR', help='save frames worth inspecting here')
    args = ap.parse_args()

    strat = strategies()
    print('decoders:', qr_utils.BACKENDS)
    if args.save_misses:
        os.makedirs(args.save_misses, exist_ok=True)

    hits = {k: 0 for k in strat}
    ms = {k: [] for k in strat}
    texts = {}
    n = any_hit = 0
    for name, img in frames(args.path, args.every):
        n += 1
        got = {}
        for k, fn in strat.items():
            t = time.perf_counter()
            res = fn(img)
            ms[k].append((time.perf_counter() - t) * 1000)
            got[k] = bool(res)
            hits[k] += got[k]
            for r in res:
                texts[r] = texts.get(r, 0) + 1
        any_hit += any(got.values())
        if args.save_misses:
            if not any(got.values()):
                cv2.imwrite(os.path.join(args.save_misses, f'none_{name}.jpg'), img)
            elif not got['current fast pass']:
                cv2.imwrite(os.path.join(args.save_misses, f'others_only_{name}.jpg'), img)
        if n % 50 == 0:
            print(f'  {n} frames...', file=sys.stderr)

    if n == 0:
        raise SystemExit('No frames found.')
    print(f'\n{n} frames, {any_hit} decoded by at least one strategy '
          f'({100 * any_hit / n:.0f}%)\n')
    print(f'{"strategy":<22}{"decoded":>10}{"of found":>10}{"ms median":>11}{"ms p90":>8}')
    for k in strat:
        rel = 100 * hits[k] / any_hit if any_hit else 0
        print(f'{k:<22}{100 * hits[k] / n:>9.0f}%{rel:>9.0f}%'
              f'{np.median(ms[k]):>11.1f}{np.percentile(ms[k], 90):>8.1f}')
    print('\ntexts seen:', ', '.join(f'"{t}" x{c}' for t, c in sorted(texts.items())) or 'none')


if __name__ == '__main__':
    main()
