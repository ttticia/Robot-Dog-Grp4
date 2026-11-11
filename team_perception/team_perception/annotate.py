"""
Annotate photos and videos with QR and/or person detections, offline (no ROS graph).

Uses the same code as the nodes (qr_utils.decode_qr, human_utils.HumanDetector), so
what you see here is what the robot would detect.

    ros2 run team_perception annotate --type qr     team_perception/test/QR.mp4
    ros2 run team_perception annotate --type human  photo.jpg photos_folder/
    ros2 run team_perception annotate --type both   clip.mp4 --name hallway
    ros2 run team_perception annotate --type qr     /mnt/c/qr_clip --frames-as-video --fps 30

Each call creates one run folder, sorted by test type:
    test_runs/<qr|human|both>/<YYYYmmdd_HHMMSS>_<name>/
        input/                   copies of the input media (skip with --no-copy)
        annotated_<video>.mp4    video with boxes, labels and a status bar
        photos/<name>.jpg        annotated photos
        first_seen/              QR: annotated frame where each code was first read
        detections.csv           one row per detection (kind = qr or person)
        summary.txt, summary.json
test_runs/ defaults to src/team_perception/test_runs in this workspace; override with
--out-root or the TEAM_PERCEPTION_TEST_RUNS environment variable.
"""
import argparse
import csv
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import time

import cv2

from team_perception import qr_utils
from team_perception.human_utils import draw_humans
from team_perception.qr_utils import decode_qr, draw_detections

TYPES = ('qr', 'human', 'both')
VIDEO_EXT = {'.mp4', '.mov', '.avi', '.mkv', '.webm', '.m4v'}
IMAGE_EXT = {'.jpg', '.jpeg', '.png', '.bmp', '.tif', '.tiff', '.webp'}


# --- locations --------------------------------------------------------------------------
def default_out_root():
    env = os.environ.get('TEAM_PERCEPTION_TEST_RUNS')
    if env:
        return os.path.expanduser(env)
    # Walk up from the cwd and from this file (works from src/ and from install/).
    for start in (os.getcwd(), os.path.dirname(os.path.abspath(__file__))):
        d = start
        while True:
            for cand in (os.path.join(d, 'src', 'team_perception'),
                         os.path.join(d, 'team_perception')):
                if os.path.isfile(os.path.join(cand, 'package.xml')):
                    return os.path.join(cand, 'test_runs')
            parent = os.path.dirname(d)
            if parent == d:
                break
            d = parent
    return os.path.expanduser('~/team_perception_test_runs')


def slug(text, n=40):
    return re.sub(r'[^A-Za-z0-9_.-]+', '_', text).strip('_')[:n] or 'run'


# --- inputs -----------------------------------------------------------------------------
def ext(path):
    return os.path.splitext(path)[1].lower()


def list_images(folder):
    return sorted(f for f in glob.glob(os.path.join(folder, '*')) if ext(f) in IMAGE_EXT)


def collect_inputs(paths, frames_as_video):
    """Return [(kind, path)] with kind 'video', 'frames' (folder played as video) or 'image'."""
    out = []
    for p in paths:
        if not os.path.exists(p):
            raise SystemExit(f'No such file or folder: {p}')
        if os.path.isdir(p):
            imgs = list_images(p)
            if imgs and frames_as_video:
                out.append(('frames', p))
            elif imgs:
                out += [('image', f) for f in imgs]
            else:
                vids = [v for v in sorted(glob.glob(os.path.join(p, '*'))) if ext(v) in VIDEO_EXT]
                if not vids:
                    print(f'warning: no images or videos in {p}', file=sys.stderr)
                out += [('video', v) for v in vids]
        elif ext(p) in VIDEO_EXT:
            out.append(('video', p))
        elif ext(p) in IMAGE_EXT:
            out.append(('image', p))
        else:
            raise SystemExit(f'Unsupported file type: {p}')
    return out


def copy_inputs(inputs, run_dir):
    """Copy every input into run_dir/input/ so the run folder is self-contained."""
    dest_dir = os.path.join(run_dir, 'input')
    os.makedirs(dest_dir, exist_ok=True)
    for _, path in inputs:
        name = os.path.basename(os.path.normpath(path))
        dest = os.path.join(dest_dir, name)
        i = 1
        while os.path.exists(dest):
            root, e = os.path.splitext(name)
            dest = os.path.join(dest_dir, f'{root}_{i}{e}')
            i += 1
        if os.path.isdir(path):
            shutil.copytree(path, dest)
        else:
            shutil.copy2(path, dest)


def read_frames(kind, path):
    if kind == 'frames':
        for f in list_images(path):
            img = cv2.imread(f)
            if img is not None:
                yield img
        return
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise SystemExit(f'Cannot open {path}')
    try:
        while True:
            ok, img = cap.read()
            if not ok:
                return
            yield img
    finally:
        cap.release()


def video_meta(kind, path, fps_arg):
    if kind == 'frames':
        return fps_arg, len(list_images(path))
    cap = cv2.VideoCapture(path)
    meta = (cap.get(cv2.CAP_PROP_FPS) or fps_arg, int(cap.get(cv2.CAP_PROP_FRAME_COUNT)))
    cap.release()
    return meta


def open_writer(path, fps, size):
    """
    Return (writer, codec).

    avc1 (H.264) plays everywhere (VS Code, browsers). The pip opencv-python wheel that
    ultralytics installs cannot encode H.264, so it falls back to mp4v, which VS Code and
    browsers cannot play; make_playable() re-encodes that afterwards.
    """
    for codec in ('avc1', 'mp4v'):
        # A failed avc1 probe prints scary but harmless [ERROR] lines from C++ straight to
        # stderr (this OpenCV build has no Python logging API), so mute fd 2 meanwhile.
        with _quiet_stderr():
            w = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*codec), fps, size)
        if w.isOpened():
            return w, codec
    raise SystemExit(f'Cannot write video {path}')


class _quiet_stderr:
    """Temporarily point file descriptor 2 at /dev/null (catches C/C++ output too)."""

    def __enter__(self):
        try:
            sys.stderr.flush()
            self.saved = os.dup(2)
            devnull = os.open(os.devnull, os.O_WRONLY)
            os.dup2(devnull, 2)
            os.close(devnull)
        except (OSError, ValueError):
            self.saved = None
        return self

    def __exit__(self, *exc):
        if self.saved is not None:
            os.dup2(self.saved, 2)
            os.close(self.saved)
        return False


def make_playable(path):
    """Re-encode an mp4v video to H.264 in place with imageio-ffmpeg's bundled ffmpeg."""
    try:
        import imageio_ffmpeg
        exe = imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        print(f'warning: {os.path.basename(path)} is MPEG-4 Part 2 (mp4v) and will not play in '
              'VS Code or browsers; pip3 install --user imageio-ffmpeg to fix', file=sys.stderr)
        return False
    tmp = path[:-4] + '.h264.mp4'
    r = subprocess.run([exe, '-y', '-loglevel', 'error', '-i', path, '-c:v', 'libx264',
                        '-preset', 'veryfast', '-crf', '23', '-pix_fmt', 'yuv420p',
                        '-movflags', '+faststart', tmp], capture_output=True, text=True)
    if r.returncode != 0 or not os.path.isfile(tmp):
        print(f'warning: H.264 re-encode failed, keeping mp4v: {r.stderr.strip()[-300:]}',
              file=sys.stderr)
        if os.path.exists(tmp):
            os.remove(tmp)
        return False
    os.replace(tmp, path)
    return True


def fit(img, max_w, max_h):
    """Downscale like the webcam sender does, so results match what the robot would get."""
    if not max_w or not max_h:
        return img
    h, w = img.shape[:2]
    s = min(max_w / w, max_h / h)
    if s >= 1:
        return img
    return cv2.resize(img, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA)


def pts_str(values):
    return ' '.join(f'{v:.0f}' for v in values)


# --- status indicator -------------------------------------------------------------------
def fmt_s(s):
    s = int(round(s))
    return f'{s // 3600}h{s // 60 % 60:02d}m' if s >= 3600 else f'{s // 60}m{s % 60:02d}s'


class Progress:
    """
    Live status of a run, shown in two places.

    - Terminal: a progress line (input i/N, frames, %, speed, ETA, detections so far),
      redrawn in place on a terminal, or one line every 10 s when output is redirected.
    - run_dir/STATUS.txt: RUNNING / DONE / FAILED / INTERRUPTED plus the same progress and
      a list of finished steps, so a run can be checked from the IDE or another shell.
    """

    def __init__(self, run_dir, opts, n_inputs, stream=None):
        self.path = os.path.join(run_dir, 'STATUS.txt')
        self.type = opts.type
        self.n_inputs = n_inputs
        self.stream = stream or sys.stdout
        try:
            self.tty = self.stream.isatty()
        except Exception:
            self.tty = False
        self.t0 = time.time()
        self.started = time.strftime('%Y-%m-%d %H:%M:%S')
        self.steps = []
        self.current = 'starting'
        self.progress = ''
        self._line_open = False
        self._last_print = self._last_file = 0.0
        self._write('RUNNING')

    def _write(self, state, note=''):
        lines = [f'State:    {state}' + (f'   ({note})' if note else ''),
                 f'Type:     {self.type}',
                 f'Started:  {self.started}   elapsed {fmt_s(time.time() - self.t0)}',
                 f'Now:      {self.current}']
        if self.progress and state == 'RUNNING':
            lines.append(f'Progress: {self.progress}')
        lines += [f'Updated:  {time.strftime("%H:%M:%S")}', '', 'Steps:']
        lines += [f'  [done] {s}' for s in self.steps] or ['  (none yet)']
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        os.replace(tmp, self.path)       # atomic, so readers never see a half-written file

    def _print(self, text, inline=False):
        if self._line_open:
            self.stream.write('\n')
            self._line_open = False
        if inline and self.tty:
            self.stream.write('\r' + text.ljust(110)[:160])
            self._line_open = True
        else:
            self.stream.write(text + '\n')
        self.stream.flush()

    def stage(self, msg):
        """Start a new step (e.g. 'loading YOLO', 'input 2/5: clip.mp4')."""
        self.current, self.progress = msg, ''
        self._print(msg)
        self._write('RUNNING')

    def done(self, msg):
        """Record a finished step."""
        self.steps.append(msg)
        self._print(f'  done: {msg}')
        self._write('RUNNING')

    def update(self, done, total, extra=''):
        """Report frame progress for the current video (throttled)."""
        now = time.time()
        if done < total and now - self._last_print < (0.5 if self.tty else 10.0):
            return
        el = now - self._step_t0 if hasattr(self, '_step_t0') else now - self.t0
        rate = done / el if el > 0 else 0.0
        pct = 100.0 * done / total if total else 0.0
        eta = fmt_s((total - done) / rate) if rate > 0 and total else '?'
        self.progress = (f'{done}/{total} frames ({pct:4.1f}%)  {rate:4.1f} fps  ETA {eta}'
                         + (f'  | {extra}' if extra else ''))
        self._last_print = now
        self._print('  ' + self.progress, inline=True)
        if now - self._last_file >= 2.0 or done >= total:
            self._last_file = now
            self._write('RUNNING')

    def start_timer(self):
        self._step_t0 = time.time()

    def finish(self, state, note=''):
        self.current = {'DONE': 'finished'}.get(state, self.current)
        self._write(state, note)
        self._print(f'Status: {state} after {fmt_s(time.time() - self.t0)}'
                    + (f' ({note})' if note else ''))


# --- processing -------------------------------------------------------------------------
class Tracker:
    """Per-input statistics for one detector type."""

    def __init__(self):
        self.frames_run = 0
        self.frames_hit = 0
        self.ms_sum = 0.0
        self.ms_max = 0.0
        self.first = None
        self.last = None

    def add(self, ms, hit, t):
        self.frames_run += 1
        self.ms_sum += ms
        self.ms_max = max(self.ms_max, ms)
        if hit:
            self.frames_hit += 1
            self.first = t if self.first is None else self.first
            self.last = t

    def stats(self, prefix):
        n = max(self.frames_run, 1)
        return {f'{prefix}_frames_run': self.frames_run,
                f'{prefix}_frames_hit': self.frames_hit,
                f'{prefix}_hit_rate': round(self.frames_hit / n, 4),
                f'{prefix}_first_s': None if self.first is None else round(self.first, 2),
                f'{prefix}_last_s': None if self.last is None else round(self.last, 2),
                f'{prefix}_ms_mean': round(self.ms_sum / n, 1),
                f'{prefix}_ms_max': round(self.ms_max, 1)}


def process_video(kind, path, run_dir, out_name, rows, opts, detector, progress):
    fps, total = video_meta(kind, path, opts.fps)
    if opts.max_frames:
        total = min(total, opts.max_frames)
    do_qr, do_human = opts.type in ('qr', 'both'), opts.type in ('human', 'both')
    src = os.path.basename(path)
    # Written under a .partial name and renamed only when finished and playable, so a video
    # that is still being written is never mistaken for a result.
    partial = os.path.join(run_dir, out_name[:-4] + '.partial.mp4')
    writer, codec = None, None
    progress.start_timer()
    qr, hu = Tracker(), Tracker()
    codes = {}                       # text -> {'first','last','frames'}
    qr_dets, qr_t = [], -1e9
    hu_dets, hu_t = [], -1e9
    max_people = 0
    n = 0
    t0 = time.time()
    for frame in read_frames(kind, path):
        if opts.max_frames and n >= opts.max_frames:
            break
        img = fit(frame, opts.max_width, opts.max_height)
        if writer is None and not opts.no_video:
            writer, codec = open_writer(partial, fps, (img.shape[1], img.shape[0]))
        t = n / fps
        status_bits = []
        new_codes = []

        if do_qr:
            full = opts.full_scan_every > 0 and n % opts.full_scan_every == 0
            s = time.time()
            found = decode_qr(img, try_upscale=full, try_clahe=full)
            ms = (time.time() - s) * 1000
            qr.add(ms, bool(found), t)
            if found:
                qr_dets, qr_t = found, t
            for text, c in found:
                if text not in codes:
                    codes[text] = {'first': t, 'last': t, 'frames': 0}
                    new_codes.append(text)
                codes[text]['last'] = t
                codes[text]['frames'] += 1
                rows.writerow([src, n, f'{t:.3f}', 'qr', text, '',
                               'full' if full else 'fast', f'{ms:.1f}', pts_str(c.ravel())])
            status_bits.append(f'qr {ms:3.0f} ms{" full" if full else ""}, codes {len(codes)}')

        if do_human:
            ms = None
            if n % opts.every == 0:
                s = time.time()
                hu_dets, hu_t = detector.detect(img), t
                ms = (time.time() - s) * 1000
                hu.add(ms, bool(hu_dets), t)
                max_people = max(max_people, len(hu_dets))
                for box, conf in hu_dets:
                    rows.writerow([src, n, f'{t:.3f}', 'person', 'person', f'{conf:.3f}', '',
                                   f'{ms:.1f}', pts_str(box)])
            shown = hu_dets if t - hu_t <= opts.hold_s else []
            status_bits.append('yolo ' + (f'{ms:3.0f} ms' if ms is not None else '(held)')
                               + f', people {len(shown)}')

        status = [f'{src}  t={t:6.1f}s  frame {n}/{total} | ' + ' | '.join(status_bits)]
        if do_qr:
            recent = sorted(codes.items(), key=lambda kv: -kv[1]['last'])[:4]
            status += [f'  QR {k[:40]}  (last {t - v["last"]:.0f}s ago)' for k, v in recent]
        if do_human:
            draw_humans(img, shown, None, None if do_qr else status, len(status))
        if do_qr:
            shown_qr = qr_dets if t - qr_t < opts.hold_s else []
            labels = {txt: [f'first seen {codes[txt]["first"]:.1f}s, '
                            f'read in {codes[txt]["frames"]} frames'] for txt, _ in shown_qr}
            draw_detections(img, shown_qr, labels, status)
        if writer is not None:
            writer.write(img)
        for text in new_codes:
            d = os.path.join(run_dir, 'first_seen')
            os.makedirs(d, exist_ok=True)
            cv2.imwrite(os.path.join(d, f'{slug(src, 20)}_t{t:07.2f}s_{slug(text, 30)}.jpg'),
                        img)
        n += 1
        found = []
        if do_qr:
            found.append(f'QR codes {len(codes)}')
        if do_human:
            found.append(f'people now {len(shown)}, max {max_people}')
        progress.update(n, total, ', '.join(found))
    progress.update(n, n)                # final 100% line even if the frame count was off
    if writer is not None:
        writer.release()
        if codec != 'avc1':
            progress.stage('  re-encoding video to H.264 ...')
            if make_playable(partial):
                codec = 'avc1 (re-encoded)'
        os.replace(partial, os.path.join(run_dir, out_name))

    res = {'input': os.path.abspath(path),
           'type': 'video' if kind == 'video' else 'frame folder',
           'output': None if opts.no_video else out_name, 'codec': codec,
           'fps': round(fps, 3), 'frames': n,
           'wall_s': round(time.time() - t0, 1)}
    if do_qr:
        res.update(qr.stats('qr'))
        res['qr_codes'] = [{'text': k, 'first_s': round(v['first'], 2),
                            'last_s': round(v['last'], 2), 'frames': v['frames']}
                           for k, v in sorted(codes.items(), key=lambda kv: kv[1]['first'])]
    if do_human:
        res.update(hu.stats('person'))
        res['max_people'] = max_people
    return res


def process_image(path, run_dir, rows, opts, detector):
    img = cv2.imread(path)
    res = {'input': os.path.abspath(path), 'type': 'photo'}
    if img is None:
        res['error'] = 'unreadable'
        return res
    img = fit(img, opts.max_width, opts.max_height)
    src = os.path.basename(path)
    status = [src]
    people, codes = [], []
    if opts.type in ('human', 'both'):
        s = time.time()
        people = detector.detect(img)
        ms = (time.time() - s) * 1000
        for box, conf in people:
            rows.writerow([src, '', '', 'person', 'person', f'{conf:.3f}', '', f'{ms:.1f}',
                           pts_str(box)])
        status.append(f'yolo {ms:.0f} ms, people {len(people)}')
        res.update(person_ms=round(ms, 1), people=len(people),
                   person_confidences=[round(c, 3) for _, c in people])
    if opts.type in ('qr', 'both'):
        s = time.time()
        codes = decode_qr(img)        # photos always get the full scan
        ms = (time.time() - s) * 1000
        for text, c in codes:
            rows.writerow([src, '', '', 'qr', text, '', 'full', f'{ms:.1f}', pts_str(c.ravel())])
        status.append(f'qr {ms:.0f} ms, codes {len(codes)}')
        res.update(qr_ms=round(ms, 1), qr_codes=[t for t, _ in codes])
    header = [' | '.join(status) + ('' if people or codes else '  (NOTHING FOUND)')]
    if opts.type in ('human', 'both'):
        draw_humans(img, people, None, None if opts.type == 'both' else header, len(header))
    if opts.type in ('qr', 'both'):
        draw_detections(img, codes, None, header)
    os.makedirs(os.path.join(run_dir, 'photos'), exist_ok=True)
    out = os.path.join('photos', os.path.splitext(src)[0] + '.jpg')
    cv2.imwrite(os.path.join(run_dir, out), img)
    res['output'] = out
    return res


# --- summary ----------------------------------------------------------------------------
def write_summary(run_dir, results, opts, argv):
    settings = {k: v for k, v in vars(opts).items() if k != 'inputs'}
    meta = {'command': ' '.join(argv), 'created': time.strftime('%Y-%m-%d %H:%M:%S'),
            'test_type': opts.type, 'qr_backends': qr_utils.BACKENDS,
            'settings': settings, 'results': results}
    with open(os.path.join(run_dir, 'summary.json'), 'w') as f:
        json.dump(meta, f, indent=2)

    do_qr, do_human = opts.type in ('qr', 'both'), opts.type in ('human', 'both')
    lines = [f'Test run {meta["created"]}   type: {opts.type}', f'command:  {meta["command"]}']
    if do_qr:
        lines.append(f'qr:       {qr_utils.BACKENDS}, full_scan_every={opts.full_scan_every}')
    if do_human:
        lines.append(f'human:    model={opts.model} conf={opts.conf} imgsz={opts.imgsz} '
                     f'device={opts.device} every={opts.every}')
    lines += [f'video:    hold_s={opts.hold_s} max_size={opts.max_width}x{opts.max_height}'
              + (f' max_frames={opts.max_frames}' if opts.max_frames else ''), '']

    for r in results:
        if r['type'] == 'photo':
            continue
        lines += [f'{r["type"].upper()}: {r["input"]}',
                  f'  output: {r["output"]}   ({r["frames"]} frames, took {r["wall_s"]} s)']
        if do_qr:
            lines.append(f'  QR:     {r["qr_frames_hit"]}/{r["qr_frames_run"]} frames with a code '
                         f'({100 * r["qr_hit_rate"]:.1f}%), decode mean {r["qr_ms_mean"]} ms, '
                         f'max {r["qr_ms_max"]} ms')
            if not r['qr_codes']:
                lines.append('          NO CODES FOUND')
            for c in r['qr_codes']:
                lines.append(f'          {c["first_s"]:7.2f}s - {c["last_s"]:7.2f}s '
                             f'{c["frames"]:5d} frames  {c["text"]}')
        if do_human:
            lines.append(f'  People: {r["person_frames_hit"]}/{r["person_frames_run"]} frames '
                         f'with a person ({100 * r["person_hit_rate"]:.1f}%), up to '
                         f'{r["max_people"]} at once, seen {r["person_first_s"]}s to '
                         f'{r["person_last_s"]}s, yolo mean {r["person_ms_mean"]} ms, '
                         f'max {r["person_ms_max"]} ms')
        lines.append('')

    photos = [r for r in results if r['type'] == 'photo']
    if photos:
        lines.append(f'PHOTOS ({len(photos)}):')
        if do_qr:
            lines.append(f'  with a QR code: {sum(1 for r in photos if r.get("qr_codes"))}')
        if do_human:
            lines.append(f'  with a person:  {sum(1 for r in photos if r.get("people"))}')
        for r in photos:
            if 'error' in r:
                desc = r['error']
            else:
                bits = []
                if do_qr:
                    bits.append('QR: ' + (' | '.join(r['qr_codes']) or 'none'))
                if do_human:
                    bits.append(f'people: {r["people"]} '
                                + ' '.join(f'{c:.2f}' for c in r['person_confidences']))
                desc = '   '.join(bits)
            lines.append(f'  {os.path.basename(r["input"]):30s} {desc}')
    text = '\n'.join(lines) + '\n'
    with open(os.path.join(run_dir, 'summary.txt'), 'w') as f:
        f.write(text)
    return text


# --- CLI --------------------------------------------------------------------------------
def parse_args(argv):
    ap = argparse.ArgumentParser(
        description='Annotate photos/videos with QR and/or person detections. Results go to '
                    'test_runs/<type>/<date>_<time>_<name>/.')
    ap.add_argument('-t', '--type', required=True, choices=TYPES,
                    help='what to detect: qr, human, or both')
    ap.add_argument('inputs', nargs='+', help='video files, photos, or folders of either')
    ap.add_argument('--name', help='run folder label (default: first input name)')
    ap.add_argument('--out-root', help='test_runs folder (default: src/team_perception/test_runs)')
    ap.add_argument('--no-copy', action='store_true',
                    help='do not copy the input media into the run folder')
    g = ap.add_argument_group('video')
    g.add_argument('--hold-s', type=float, default=0.5,
                   help='keep drawing boxes this long after the last detection')
    g.add_argument('--frames-as-video', action='store_true',
                   help='treat image folders as video frames (e.g. from --record)')
    g.add_argument('--fps', type=float, default=30.0, help='frame rate for --frames-as-video')
    g.add_argument('--max-frames', type=int, default=0,
                   help='stop each video after this many frames (0 = all; for quick tests)')
    g.add_argument('--no-video', action='store_true', help='skip the annotated video (stats only)')
    g.add_argument('--max-width', type=int, default=1280,
                   help='downscale larger inputs like the webcam sender (0 = never)')
    g.add_argument('--max-height', type=int, default=720)
    q = ap.add_argument_group('QR')
    q.add_argument('--full-scan-every', type=int, default=5,
                   help='every Nth video frame also tries CLAHE + 2x upscale, like qr_scanner '
                        '(0 = never, 1 = every frame); photos always get the full scan')
    h = ap.add_argument_group('human')
    h.add_argument('--model', default='yolo11n.pt',
                   help='YOLO weights: bare name (kept in ~/.cache/team_perception) or a path')
    h.add_argument('--conf', type=float, default=0.4, help='minimum person confidence')
    h.add_argument('--imgsz', type=int, default=640, help='YOLO input size')
    h.add_argument('--device', default='cpu', help="'cpu' or '0' for the first GPU")
    h.add_argument('--every', type=int, default=1,
                   help='run YOLO on every Nth video frame (boxes are held in between)')
    return ap.parse_args(argv)


def run(argv, detector=None):
    """Process the inputs named in argv and return the run folder path."""
    opts = parse_args(argv)
    opts.every = max(1, opts.every)
    inputs = collect_inputs(opts.inputs, opts.frames_as_video)
    if not inputs:
        raise SystemExit('Nothing to process.')
    root = opts.out_root or default_out_root()
    name = opts.name or os.path.splitext(os.path.basename(os.path.normpath(opts.inputs[0])))[0]
    run_dir = os.path.join(root, opts.type, f'{time.strftime("%Y%m%d_%H%M%S")}_{slug(name)}')
    os.makedirs(run_dir)
    print(f'Test type: {opts.type}   inputs: {len(inputs)}\nWriting to {run_dir}\n'
          f'Status file: {os.path.join(run_dir, "STATUS.txt")}')
    progress = Progress(run_dir, opts, len(inputs))
    try:
        _run_steps(opts, argv, inputs, run_dir, detector, progress)
    except KeyboardInterrupt:
        progress.finish('INTERRUPTED', 'stopped with Ctrl+C; results are incomplete')
        raise
    except BaseException as e:
        progress.finish('FAILED', f'{type(e).__name__}: {e}')
        raise
    progress.finish('DONE', f'results in {run_dir}')
    return run_dir


def _run_steps(opts, argv, inputs, run_dir, detector, progress):
    if not opts.no_copy:
        progress.stage(f'copying {len(inputs)} input(s) into input/ ...')
        copy_inputs(inputs, run_dir)
        progress.done(f'copied {len(inputs)} input(s) into input/')
    if opts.type in ('human', 'both') and detector is None:
        progress.stage(f'loading YOLO model {opts.model} on {opts.device} ...')
        t = time.time()
        from team_perception.human_utils import HumanDetector
        detector = HumanDetector(opts.model, opts.conf, opts.imgsz, opts.device)
        progress.done(f'loaded YOLO {opts.model} ({time.time() - t:.1f} s)')

    results, used = [], set()
    with open(os.path.join(run_dir, 'detections.csv'), 'w', newline='') as f:
        rows = csv.writer(f)
        rows.writerow(['input', 'frame', 'time_s', 'kind', 'label', 'conf', 'scan', 'ms',
                       'points'])
        for i, (kind, path) in enumerate(inputs, 1):
            src = os.path.basename(os.path.normpath(path))
            label = {'image': 'photo', 'frames': 'frame folder'}.get(kind, kind)
            progress.stage(f'input {i}/{len(inputs)}: {src} ({label})')
            if kind == 'image':
                r = process_image(path, run_dir, rows, opts, detector)
                results.append(r)
                found = []
                if 'qr_codes' in r:
                    found.append(f'QR {len(r["qr_codes"])}')
                if 'people' in r:
                    found.append(f'people {r["people"]}')
                summary = ', '.join(found) or r.get('error', '')
                progress.done(f'{i}/{len(inputs)} {src}: {summary}')
                continue
            base = os.path.splitext(src)[0]
            out = f'annotated_{slug(base)}.mp4'
            while out in used:
                out = out.replace('.mp4', '_1.mp4')
            used.add(out)
            r = process_video(kind, path, run_dir, out, rows, opts, detector, progress)
            results.append(r)
            progress.done(f'{i}/{len(inputs)} {src}: {r["frames"]} frames in '
                          f'{fmt_s(r["wall_s"])} -> {out}')

    progress.stage('writing summary ...')
    text = write_summary(run_dir, results, opts, ['annotate'] + list(argv))
    progress.done('wrote summary.txt / summary.json')
    print()
    print(text)


def main():
    run(sys.argv[1:])


if __name__ == '__main__':
    main()
