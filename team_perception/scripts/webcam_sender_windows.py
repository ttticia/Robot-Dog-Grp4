r"""
Send the Windows laptop webcam to WSL (run this on WINDOWS, not in WSL).

WSL2 cannot see built-in (MIPI) laptop cameras, so this script reads the camera
on Windows and streams JPEG frames over TCP to the `webcam_bridge` ROS node
running in WSL, which republishes them as a sensor_msgs/Image topic.

Windows forwards localhost to WSL, so no IP address or firewall rule is needed.

Setup (PowerShell, once):   pip install opencv-python numpy
Run:                         python webcam_sender_windows.py [--camera 0] [--port 5005]
Start `ros2 run team_perception webcam_bridge` in WSL first (or after - it retries).
Ctrl+C to stop.

Record a test clip (saves exactly the JPEG frames the scanner receives):
    python webcam_sender_windows.py --record C:\qr_clip --no-stream --preview
then evaluate it in WSL:  python3 scripts/eval_qr_video.py /mnt/c/qr_clip

Play a video file (or a folder of .jpg/.png frames) instead of the camera, in real
time, so the whole ROS pipeline and the rqt overlay can be tested on the same clip:
    python webcam_sender_windows.py --video C:\Users\me\Videos\qr_test.mp4 --loop
This also works inside WSL (no Windows needed), e.g.
    python3 scripts/webcam_sender_windows.py --video /mnt/c/qr_clip --speed 0.5
Frames larger than --width x --height are scaled down (phone 4K -> 1280x720).
"""
import argparse
import glob
import os
import socket
import struct
import threading
import time

import cv2

# Sent once per connection so the bridge can ignore anything else that connects to
# the port (e.g. VS Code port-forwarding probes or a browser). Must match the bridge.
MAGIC = b'QRCAM1\n'


def open_camera(index, width, height, fps):
    # CAP_DSHOW opens built-in Windows cameras faster and more reliably than the default.
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(index)
    # Many webcams only reach 30 fps at 720p in MJPG; the default (YUY2) is often 5-10 fps.
    # FOURCC must be set before the size for DirectShow to honour it.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # don't queue old frames inside the driver
    return cap


class LatestFrame:
    """
    Reads the camera on its own thread and keeps only the newest frame.

    Without this, frames queue up while we encode/send, and the video falls behind.
    """

    def __init__(self, cap):
        self.cap = cap
        self.cond = threading.Condition()
        self.frame, self.seq, self.ok = None, 0, True
        self.grabbed, self.t0 = 0, time.time()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while self.ok:
            ok, frame = self.cap.read()
            with self.cond:
                if not ok:
                    self.ok = False
                else:
                    self.frame, self.seq = frame, self.seq + 1
                    self.grabbed += 1
                self.cond.notify_all()

    def camera_fps(self):
        fps = self.grabbed / max(time.time() - self.t0, 1e-6)
        self.grabbed, self.t0 = 0, time.time()
        return fps

    def get(self, last_seq):
        """Block until a frame newer than last_seq exists; return (frame, seq) or (None, _)."""
        with self.cond:
            self.cond.wait_for(lambda: self.seq > last_seq or not self.ok, timeout=2.0)
            if not self.ok or self.seq <= last_seq:
                return None, last_seq
            return self.frame, self.seq


class VideoFrames:
    """
    Plays a video file or a folder of frames with the same get() interface as LatestFrame.

    Frames are paced to the clip's own frame rate (times `speed`). If sending falls
    behind, frames are skipped, like a live camera, so the scanner sees real-time video.
    """

    def __init__(self, path, loop=False, speed=1.0, folder_fps=30.0, max_size=None):
        if not os.path.exists(path):
            raise SystemExit(f'No such file or folder: {path}')
        self.files = None
        if os.path.isdir(path):
            self.files = sorted(glob.glob(os.path.join(path, '*.jpg')) +
                                glob.glob(os.path.join(path, '*.png')))
            if not self.files:
                raise SystemExit(f'No .jpg/.png frames in {path}')
            self.fps = folder_fps
            self.count = len(self.files)
        else:
            self.cap = cv2.VideoCapture(path)
            if not self.cap.isOpened():
                raise SystemExit(f'Cannot open video {path}')
            fps = self.cap.get(cv2.CAP_PROP_FPS)
            self.fps = fps if 1 <= fps <= 240 else 30.0
            self.count = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.path, self.loop, self.speed, self.max_size = path, loop, speed, max_size
        self.ok, self.seq, self.pos = True, 0, 0     # pos = index of the next frame to read
        self.played, self.t_stats = 0, time.time()
        self._restart()

    def describe(self):
        kind = 'frames' if self.files else 'video'
        return (f'Playing {kind} {self.path}: {self.count} frames at {self.fps:.1f} fps '
                f'x{self.speed:g}{" (looping)" if self.loop else ""}')

    def _restart(self):
        if self.files is None:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        self.pos, self.t0 = 0, time.time()

    def _read(self, decode=True):
        if self.files is not None:
            if self.pos >= len(self.files):
                return False, None
            self.pos += 1
            return True, (cv2.imread(self.files[self.pos - 1]) if decode else None)
        ok = self.cap.read() if decode else (self.cap.grab(), None)
        if ok[0]:
            self.pos += 1
        return ok

    def _shrink(self, frame):
        if frame is None or not self.max_size:
            return frame
        h, w = frame.shape[:2]
        k = min(self.max_size[0] / w, self.max_size[1] / h)
        if k >= 1:
            return frame
        return cv2.resize(frame, (int(w * k), int(h * k)), interpolation=cv2.INTER_AREA)

    def camera_fps(self):
        fps = self.played / max(time.time() - self.t_stats, 1e-6)
        self.played, self.t_stats = 0, time.time()
        return fps

    def get(self, last_seq):
        """Wait until the next frame is due, skip any that are overdue, return (frame, seq)."""
        rate = self.fps * self.speed
        due = int((time.time() - self.t0) * rate)          # index of the frame due now
        if due < self.pos:                                  # early: wait for the next one
            time.sleep(max(0.0, self.t0 + self.pos / rate - time.time()))
            due = self.pos
        while self.pos < due:                               # late: skip without decoding
            if not self._read(decode=False)[0]:
                break
        ok, frame = self._read()
        if not ok:
            if not self.loop:
                print('End of video.')
                self.ok = False
                return None, last_seq
            print('End of video, looping.')
            self._restart()
            ok, frame = self._read()
            if not ok:
                self.ok = False
                return None, last_seq
        self.seq += 1
        self.played += 1
        return self._shrink(frame), self.seq

    def release(self):
        if self.files is None:
            self.cap.release()


def connect(host, port):
    while True:
        try:
            s = socket.create_connection((host, port), timeout=3)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.settimeout(None)
            s.sendall(MAGIC)
            print(f'Connected to WSL bridge at {host}:{port}')
            return s
        except OSError:
            print(f'Waiting for webcam_bridge on {host}:{port} ... '
                  '(is `ros2 run team_perception webcam_bridge` running in WSL?)')
            time.sleep(2)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--camera', type=int, default=0, help='camera index (try 1 if 0 fails)')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=5005)
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--fps', type=float, default=30.0, help='max frames per second to send')
    ap.add_argument('--quality', type=int, default=70, help='JPEG quality 1-100 '
                    '(lower = smaller frames, less lag; QR still decodes fine at 60-70)')
    ap.add_argument('--preview', action='store_true', help='also show a local preview window')
    ap.add_argument('--record', metavar='FOLDER',
                    help='also save every sent frame as FOLDER/frame_00001.jpg, ... (test clips)')
    ap.add_argument('--no-stream', action='store_true',
                    help='do not connect to WSL (use with --record to only record)')
    ap.add_argument('--video', metavar='PATH',
                    help='play this video file or folder of frames instead of the camera')
    ap.add_argument('--loop', action='store_true', help='with --video: restart at the end')
    ap.add_argument('--speed', type=float, default=1.0,
                    help='with --video: playback speed (0.5 = half speed)')
    args = ap.parse_args()
    if args.record:
        os.makedirs(args.record, exist_ok=True)
        print(f'Recording frames to {os.path.abspath(args.record)}')

    if args.video:
        grabber = VideoFrames(args.video, args.loop, args.speed, args.fps,
                              (args.width, args.height))
        print(grabber.describe())
        cap = grabber
    else:
        cap = open_camera(args.camera, args.width, args.height, args.fps)
        grabber = None
    if grabber is None and not cap.isOpened():
        raise SystemExit(f'Could not open camera {args.camera}. Close other apps using the '
                         'camera (Teams, Zoom, Camera app) and check Windows camera privacy '
                         'settings ("Let desktop apps access your camera").')
    if grabber is None:
        ok, frame = cap.read()
        if not ok:
            raise SystemExit('Camera opened but returned no frames.')
        fourcc = int(cap.get(cv2.CAP_PROP_FOURCC)).to_bytes(4, 'little').decode(errors='replace')
        print(f'Camera {args.camera}: {frame.shape[1]}x{frame.shape[0]}, format {fourcc}, '
              f'driver says {cap.get(cv2.CAP_PROP_FPS):.0f} fps')
        grabber = LatestFrame(cap)
    sock = None if args.no_stream else connect(args.host, args.port)
    # A video is already paced by VideoFrames; --fps then only applies to frame folders.
    period = 0.0 if args.video else 1.0 / args.fps
    sent, t0, seq = 0, time.time(), 0
    try:
        while True:
            start = time.time()
            frame, seq = grabber.get(seq)
            if frame is None:
                if not args.video:
                    print('Camera stopped sending frames.')
                break
            ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, args.quality])
            data = jpg.tobytes()
            if args.record:
                with open(os.path.join(args.record, f'frame_{seq:05d}.jpg'), 'wb') as f:
                    f.write(data)
            if sock is not None:
                try:
                    sock.sendall(struct.pack('>I', len(data)) + data)
                except OSError:
                    print('Connection to WSL lost, reconnecting...')
                    sock.close()
                    sock = connect(args.host, args.port)
                    continue

            sent += 1
            if time.time() - t0 >= 5:
                print(f'{"video" if args.video else "camera"} {grabber.camera_fps():.1f} fps, '
                      f'sending {sent / (time.time() - t0):.1f} fps, '
                      f'{len(data) / 1024:.0f} KB/frame')
                sent, t0 = 0, time.time()
            if args.preview:
                cv2.imshow('webcam sender (q to quit)', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            time.sleep(max(0.0, period - (time.time() - start)))
    except KeyboardInterrupt:
        pass
    finally:
        if grabber is not None:
            grabber.ok = False
        cap.release()
        if sock is not None:
            sock.close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
