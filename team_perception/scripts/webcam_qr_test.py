"""
Live webcam test for qr_utils.decode_qr (no ROS needed).

Usage (from the team_perception folder):
    python3 scripts/webcam_qr_test.py            # camera 0
    python3 scripts/webcam_qr_test.py 1          # another camera index
Press q (or Esc) in the video window to quit.
"""
import os
import sys
import time

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from team_perception import qr_utils  # noqa: E402
from team_perception.qr_utils import decode_qr  # noqa: E402


def main():
    index = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        sys.exit(f'Could not open camera {index}. In WSL check that /dev/video{index} exists.')

    print('Decoder:', qr_utils.BACKENDS)
    seen = set()
    last = time.time()
    fps = 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            print('Camera stopped sending frames.')
            break

        codes = decode_qr(frame)
        for text, corners in codes:
            pts = corners.astype(int)
            cv2.polylines(frame, [pts], True, (0, 255, 0), 3)
            cv2.putText(frame, text[:40], tuple(pts[0]), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 0), 2)
            if text not in seen:
                seen.add(text)
                u, v = corners.mean(axis=0)
                print(f'NEW QR: "{text}" at pixel ({u:.0f}, {v:.0f})')

        now = time.time()
        fps = 0.9 * fps + 0.1 / max(now - last, 1e-6)
        last = now
        cv2.putText(frame, f'{fps:.1f} fps  codes: {len(codes)}', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.imshow('QR webcam test (q to quit)', frame)
        if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
            break

    cap.release()
    cv2.destroyAllWindows()
    print('Codes seen:', sorted(seen))


if __name__ == '__main__':
    main()
