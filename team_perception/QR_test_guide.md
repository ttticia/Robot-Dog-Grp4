# QR Scanner: Setup, Testing and How It Works

The `team_perception` package finds QR codes in a camera stream. For each new code it:
- publishes the text (and the code's map position, if depth is available) on `/qr/detections`
- saves a snapshot and a CSV log line as evidence
- shows a live video with boxes and text, for viewing in rqt

Contents:
1. [Install](#1-install-once-per-machine)
2. [Build](#2-build)
3. [Unit tests](#3-run-the-unit-tests)
4. [Test with the laptop webcam (WSL)](#4-test-with-the-laptop-webcam-wsl)
5. [Run on the robot](#5-run-on-the-robot)
6. [Parameters](#6-scanner-parameters)
7. [Output format](#7-output)
8. [How it works](#8-how-it-works)
9. [Testing on video](#9-testing-on-video)
10. [Tips and known limits](#10-tips-and-known-limits)
11. [Troubleshooting](#11-troubleshooting)
12. [Sharing changes (git)](#12-sharing-changes-git)

---

## 1. Install (once per machine)

ROS 2 Humble on Ubuntu 22.04 (native or WSL2).

The GitHub repo **is** the workspace's `src/` folder. Clone it there:
```bash
mkdir -p ~/robot_challenge_workspace && cd ~/robot_challenge_workspace
git clone git@github.com:ttticia/Robot-Dog-Grp4.git src
```

```bash
# Decoders: Humble's OpenCV 4.5.4 is built WITHOUT QR decoding support (no QUIRC)
sudo apt install python3-pyzbar python3-qrcode ros-humble-image-transport-plugins ros-humble-rqt-image-view
```
Then do the pip installs in the [README, section 2](README.md#2-what-you-need-to-install-first). `requirements.txt` also holds the person detector's packages (ultralytics). Install CPU torch **first**, and remove pip's setuptools afterwards, as described there. Otherwise pip pulls the multi-GB CUDA torch and a setuptools that breaks colcon.

| Package | Why |
|---|---|
| `zxing-cpp` (pip) | Main decoder. Fast and good at angled codes. |
| `python3-pyzbar` | Second decoder, used when zxing misses. |
| `python3-qrcode` | Only the tests use it, to generate QR images. |
| `image-transport-plugins` | Lets rqt show the `compressed` (JPEG) topics. |

Check which decoders the scanner can see:
```bash
python3 -c "from team_perception import qr_utils; print(qr_utils.BACKENDS)"
```
The output should list both zxing-cpp and pyzbar. With neither installed, the scanner falls back to OpenCV, which cannot decode on Humble.

## 2. Build

```bash
cd ~/robot_challenge_workspace
colcon build --packages-select team_perception --symlink-install
source install/setup.bash        # already in ~/.bashrc on the dev laptop
```

The package has four executables. This guide covers the QR side:
- `qr_scanner` is the scanner node.
- `webcam_bridge` receives the Windows webcam stream (see section 4).
- `human_detector` is the YOLO person node (see the [README](README.md), section 6).
- `annotate` is the offline photo and video annotation tool (see the end of section 9).

## 3. Run the unit tests

```bash
cd ~/robot_challenge_workspace/src/team_perception
python3 -m pytest test/ -v
# or, the ROS way (also runs flake8 / pep257 style checks):
cd ~/robot_challenge_workspace && colcon test --packages-select team_perception && colcon test-result --verbose
```

Expected result: no failures. One result doesn't count as a pass: `test_real_photo_curved_paper` is **xfail** (an expected failure). No decoder can read `test_D.jpg` because the paper is curved (see section 10).

Run `colcon` from the workspace root (`~/robot_challenge_workspace`), never from `src/`. Run from `src/`, it creates `build/`, `install/` and `log/` folders inside the git repo.

What the tests cover:
- Codes that the tests generate: plain, small inside a large scene, perspective-tilted, and no code at all.
- The real photo `test/testing_data/test_images/test_c.jpg`: a hand-held printout seen at an angle, which must decode as `C`.
- The scanner still working without zxing (pyzbar only).
- The "flatten a located but unreadable code" fallback.
- Depth lookup, 3-D deprojection, and the overlay drawing.

Do not use `pytest -s` or run Python with `-s`. Those flags skip `~/.local`, where pip put zxing-cpp, so the zxing tests are skipped.

## 4. Test with the laptop webcam (WSL)

WSL2 cannot see a built-in laptop camera: it is a MIPI device, not USB, so `usbipd` does not help. Instead, a small script on Windows streams the camera to WSL over TCP.

```
Windows: webcam_sender_windows.py ──TCP:5005 (JPEG)──▶ WSL: webcam_bridge ──▶ /camera/color/image_raw[/compressed]
                                                                                       │
                                                       rqt_image_view ◀── /qr/debug_image/compressed ◀── qr_scanner
```

**Windows (PowerShell), once:**
```powershell
pip install opencv-python numpy
copy \\wsl$\Ubuntu-22.04\home\<you>\robot_challenge_workspace\src\team_perception\scripts\webcam_sender_windows.py C:\qr\
```
Replace `<you>` with your WSL user name.

**Terminal 1 (WSL): bridge**
```bash
ros2 run team_perception webcam_bridge
```

**Terminal 2 (Windows PowerShell): sender**
```powershell
python C:\qr\webcam_sender_windows.py            # add --camera 1 if 0 is the wrong camera, --preview for a local window
```
Windows forwards `localhost` to WSL, so the sender needs no IP address. The bridge logs its frame rate once frames arrive.

**Terminal 3 (WSL): scanner**
```bash
ros2 run team_perception qr_scanner --ros-args -p image_transport:=compressed -p use_depth:=false
```
Use `image_transport:=compressed`. Raw 720p frames are 2.7 MB each, and DDS drops most of them.

**Terminal 4 (WSL): view and listen**
```bash
ros2 run rqt_image_view rqt_image_view /qr/debug_image/compressed   # opens on Windows via WSLg
ros2 topic echo /qr/detections std_msgs/msg/String
```

Measured on the dev laptop: about 23 fps of video, with 36 to 40 ms of lag.

Sender options:

| Option | Default | Notes |
|---|---|---|
| `--camera` | 0 | Camera index. |
| `--width` / `--height` | 1280 / 720 | Lower values give more fps and less lag, but small codes are harder to read. |
| `--fps` | 30 | Maximum send rate. |
| `--quality` | 70 | JPEG quality. |
| `--preview` | off | Shows a local window. |
| `--record FOLDER` | none | Saves every sent JPEG. |
| `--no-stream` | off | Records without connecting to WSL. |
| `--host` / `--port` | 127.0.0.1 / 5005 | Must match the bridge's `port` parameter. |
| `--video PATH` | none | Plays a video file, or a folder of `.jpg`/`.png` frames, instead of the camera (see section 9). |
| `--loop` | off | With `--video`: restarts at the end. |
| `--speed` | 1.0 | With `--video`: playback speed (0.5 = half speed). |

**Quick test without ROS:** if a USB webcam is attached to WSL (it shows up as `/dev/video0`), run:
```bash
python3 scripts/webcam_qr_test.py [camera_index]
```

## 5. Run on the robot

Find the robot's real topic names first:
```bash
ros2 topic list | grep -i -e image -e camera_info -e depth
```
Then start the scanner with those names. The values below are the defaults (RealSense style):
```bash
ros2 run team_perception qr_scanner --ros-args \
  -p image_topic:=/camera/color/image_raw \
  -p image_transport:=compressed \
  -p depth_topic:=/camera/aligned_depth_to_color/image_raw \
  -p camera_info_topic:=/camera/color/camera_info \
  -p map_frame:=map
```
For the scanner to report a map position:
- The depth image must be **aligned to the colour image**.
- `camera_info` must be published.
- A TF chain must exist from the camera's optical frame to `map_frame`.

Without TF, the scanner reports the position in the camera frame and logs a warning. Without depth, it reports `no position`.

To watch from another computer, view `/qr/debug_image/compressed`. Raw images cannot keep up at 30 fps over Wi-Fi.

## 6. Scanner parameters

| Parameter | Default | Meaning |
|---|---|---|
| `image_topic` | `/camera/color/image_raw` | Colour image topic. |
| `image_transport` | `raw` | `compressed` subscribes to `<image_topic>/compressed`; recommended. |
| `depth_topic` | `/camera/aligned_depth_to_color/image_raw` | Aligned depth, in uint16 mm or float32 m. |
| `camera_info_topic` | `/camera/color/camera_info` | Intrinsics, used for 3-D positions. |
| `use_depth` | `true` | Set to `false` when there is no depth camera, e.g. the webcam. |
| `map_frame` | `map` | Frame that positions are reported in. |
| `rate_hz` | 30 | Maximum decodes per second (0 = unlimited). Lower it to save CPU. |
| `full_scan_every` | 5 | Every Nth decode also tries a contrast boost and 2× upscale, which finds small or far codes but is about 5× slower. 0 = never. |
| `hold_s` | 0.5 | Keeps drawing a box this long after the code was last decoded, so it doesn't flicker. |
| `repeat_after_s` | 30 | Reports the same text again only after this many seconds. |
| `publish_debug` | `true` | Publishes the overlay video. |
| `log_dir` | `~/qr_logs` | Where the CSV log and snapshots go. |

## 7. Output

**`/qr/detections`** (`std_msgs/String`, JSON). One message is sent per new code, and again after `repeat_after_s`:
```json
{"text": "C", "pixel": [1402.5, 690.1], "frame": "map",
 "position": [3.12, -0.84, 0.41], "snapshot": "/home/you/qr_logs/qr_1762871234567.jpg",
 "stamp": 1762871234.52}
```
- `frame` is `map`, or the camera frame if TF failed, or `none` when there is no depth.
- `position` is `null` when `frame` is `none`.

**`~/qr_logs/`** contains:
- `qr_YYYYMMDD_HHMMSS.csv`: one row per report, with the columns `wall_time, text, frame, x, y, z, snapshot`.
- `qr_<ms>.jpg`: an annotated snapshot of each report, kept as evidence.

**`/qr/debug_image`** and **`/qr/debug_image/compressed`** show the camera video with:
- a green box around each code, with a red dot on its first corner
- a label with the decoded text, where the code is, and how many times it has been seen
- a status panel with the video fps, the decode rate, and the last 5 codes

The overlay is only drawn while something subscribes to one of these topics.

## 8. How it works

**Files**

| File | Contents |
|---|---|
| `team_perception/qr_utils.py` | All the image work, with no ROS imports, so it can be tested and run without ROS. |
| `team_perception/qr_scanner_node.py` | The ROS node. |
| `team_perception/webcam_bridge_node.py` | Receives JPEG frames over TCP and publishes them as ROS images. |
| `scripts/webcam_sender_windows.py` | Streams the camera, or plays a video with `--video`, to the bridge. |
| `scripts/eval_qr_video.py` | Benchmarks the decoders on a recorded clip. |
| `scripts/webcam_qr_test.py` | Live test without ROS. |

**Decoding pipeline (`decode_qr`)**

For each frame, the image is converted to greyscale and the steps below run in order. The scanner stops at the first step that finds a code.
1. **zxing-cpp** reads the code. It is fast (about 4 ms per 720p frame) and copes well with perspective. When it *finds* a code but can't read it, it still returns the four corners.
2. **pyzbar** tries any codes zxing missed.
3. **Flattening:** for up to 3 codes that zxing located but couldn't read, the scanner uses their corners to warp the code back to a flat square (`getPerspectiveTransform`), adds a white border, and decodes it again.
4. *(Full scan only)* The scanner boosts contrast (CLAHE) and tries again.
5. *(Full scan only)* It upscales the image 2× for small or far codes and tries again. Corners are scaled back to original pixels.

OpenCV's own QR detector is used only if neither zxing nor pyzbar is installed.

**Node threading**

```
camera callback ──▶ stores latest frame, wakes decoder ──▶ (if anyone is watching) draws last boxes + publishes debug video
decode thread   ──▶ waits for a new frame (max rate_hz) ──▶ decode_qr (full scan every Nth) ──▶ for each NEW text: report()
```
Decoding runs on its own thread. The camera callback never waits for it, so the debug video stays at full camera rate even when a decode is slow. If decoding falls behind, the scanner skips old frames and always decodes the newest one, so lag doesn't build up.

**Reporting a code (`report` and `locate`)**
1. The pixel centre of the code is the mean of its 4 corners.
2. Depth is read at that pixel. This is the median of a small window, with zeros and NaNs ignored, and it handles both mm and m.
3. The pixel and depth are deprojected with the intrinsics from `camera_info` into a 3-D point in the camera optical frame.
4. TF transforms the point into `map_frame`.
5. The scanner publishes the JSON message, writes the CSV row, and saves the snapshot.

**Webcam bridge**
- The sender first sends a handshake (`QRCAM1\n`), then a stream of `[4-byte big-endian length][JPEG bytes]` frames. The bridge drops any connection that doesn't start with the handshake. For example, VS Code's port-forwarding probes used to cause "Bad frame size" errors.
- The JPEG is republished unchanged on `<topic>/compressed`, with no re-encoding.
- The bridge decodes frames to a raw `Image` only when something subscribes to the raw topic.
- The sender always sends the *newest* camera frame, so lag can't build up.

## 9. Testing on video

There are two ways to test on a clip:
- **Score it** with `eval_qr_video.py`. This gives numbers, and needs no ROS.
- **Play it through the full ROS pipeline**, which lets you watch the boxes and text in rqt.

Either way, use footage like the challenge will have: codes at different angles and distances, filmed at walking speed. Any of these sources works:
- a phone video (`.mp4` / `.mov`)
- a frame folder recorded with `--record`
- a frame folder exported from a ROS bag

From WSL, Windows files are under `/mnt/c/...`.

### Play a clip through the ROS pipeline

The sender plays the video in real time at the clip's own frame rate. If it can't keep up, it skips frames, just like a live camera. Frames larger than `--width`×`--height` (default 1280×720) are scaled down, so a 4K phone video arrives the same size as the webcam's.

The script only needs OpenCV, so it also runs inside WSL and no Windows terminal is needed:
```bash
# Terminal 1: bridge
ros2 run team_perception webcam_bridge
# Terminal 2: play the clip (file or frame folder); --loop repeats it, --speed 0.5 = slow motion
python3 ~/robot_challenge_workspace/src/team_perception/scripts/webcam_sender_windows.py \
    --video /mnt/c/Users/<you>/Videos/qr_test.mp4 --loop
# Terminal 3: scanner, and Terminal 4: rqt_image_view + topic echo, exactly as in section 4
ros2 run team_perception qr_scanner --ros-args -p image_transport:=compressed -p use_depth:=false
```
From Windows PowerShell instead, run `python C:\qr\webcam_sender_windows.py --video C:\Users\<you>\Videos\qr_test.mp4 --loop`.

For a frame folder, `--fps` sets the playback rate (default 30). A video file uses its own rate.

Replaying the same clip after each change to the scanner, or after changing a parameter such as `full_scan_every`, shows whether things got better.

### Score a clip (no ROS)

To record a clip with the laptop webcam:

```powershell
# Windows: record without streaming
python C:\qr\webcam_sender_windows.py --record C:\qr_clip --no-stream --preview
```
```bash
# WSL: score each decoding strategy on the clip
python3 scripts/eval_qr_video.py /mnt/c/qr_clip --save-misses ~/qr_misses
python3 scripts/eval_qr_video.py some_video.mp4 --every 2          # videos also work
```
The script prints, for each strategy (fast pass, full scan, zxing only, pyzbar only), the share of frames it decoded and its median and 90th-percentile time.

`--save-misses` saves two kinds of frames for you to inspect:
- `none_*.jpg`: frames where no strategy found a code.
- `others_only_*.jpg`: frames where the fast pass missed but another strategy found a code.

Benchmarks from development:
- **Tilted codes:** zxing + pyzbar read 117 of 144 synthetic angles; pyzbar alone read 113.
- **Small codes:** 69 vs 63 in the same comparison.
- **Speed:** about 4 ms per frame with a code in view, vs about 50 ms for pyzbar alone.

### Annotate photos and videos into `test_runs/` (no ROS graph)

The `annotate` tool runs the scanner's own decoder and overlay (`qr_utils.decode_qr` and `draw_detections`) on files. It saves the results so you can review or compare them later. For QR-only tests, use `--type qr`. To also draw YOLO person boxes, use `--type both`.
```bash
ros2 run team_perception annotate --type qr team_perception/test/testing_data/QR.mp4               # a video
ros2 run team_perception annotate --type qr photo1.jpg photos_folder/ --name hallway   # photos
ros2 run team_perception annotate --type qr /mnt/c/qr_clip --frames-as-video --fps 30  # recorded frame folder
```
Each call creates `team_perception/test_runs/qr/<date>_<time>_<name>/` (or `both/`) containing:
- `input/`: a copy of the input media (`--no-copy` skips this).
- `annotated_<video>.mp4`: the video with boxes, decoded text and a status bar (time, decode ms, codes found).
- `photos/*.jpg`: annotated photos. Photos where nothing was found are marked `NOTHING FOUND`.
- `first_seen/*.jpg`: the frame where each code was first read.
- `detections.csv`: one row per code per frame (time, fast/full scan, decode ms, corner points).
- `summary.txt` / `summary.json`: hit rate, decode times, when each code was seen, and the exact command and settings used.

Videos follow the node's schedule: every frame gets a fast pass, and every 5th frame gets a full scan (`--full-scan-every`). Boxes stay on screen for `--hold-s` 0.5. Photos always get the full scan. Inputs larger than 1280×720 are scaled down, as the webcam sender does (`--max-width 0` turns this off). Unlike the live node, every frame is decoded and none are dropped.

Other options: `--max-frames N`, `--no-video` and `--out-root DIR` (or the `TEAM_PERCEPTION_TEST_RUNS` environment variable). With `--type qr`, the tool takes about 4 minutes for a 2-minute 640×480 clip. See the README, section 7, for the full folder layout.

## 10. Tips and known limits

- **Keep codes flat.** Mount them on card or a rigid surface. A curved printout (`test_D.jpg`) defeats every decoder we tried: zxing, pyzbar, OpenCV, WeChat CNN and QReader. YOLO detectors (qrdet) still *find* it, though, with 0.91 to 0.93 confidence.
- **Angle:** moderate tilt is fine (see the benchmark above), but steep angles still fail. Past that point, getting the robot to face the code more directly helps more than any software change.
- **Size and distance:** a code needs about 3 or more pixels per module. A 21×21 code therefore needs at least about 80 px across in the image. The full scan (2× upscale) helps a little at longer range.
- **Lighting:** glare on glossy paper is worse than dim light. Use matte prints.
- **CPU:** if the robot computer is busy, set `rate_hz:=10` and `full_scan_every:=0`, or lower the camera resolution.
- **Planned:** a YOLO-based `qr_detector` (qrdet, exported to ONNX), to spot codes the decoders can't read yet and steer the robot towards a better view. This is not built yet.

## 11. Troubleshooting

| Symptom | Fix |
|---|---|
| Startup log says the backend is `OpenCV` only, or nothing ever decodes | Install pyzbar and zxing-cpp (section 1). Check with `qr_utils.BACKENDS`. |
| zxing tests are skipped | Run `pip3 install --user zxing-cpp==2.3.0`, and don't run Python with `-s`. |
| `rqt_image_view: command not found` | Run `sudo apt install ros-humble-rqt-image-view`. It opens on Windows through WSLg. |
| rqt has no `compressed` option | Run `sudo apt install ros-humble-image-transport-plugins`. |
| rqt video is laggy or choppy | View `/qr/debug_image/compressed`, and run the scanner with `image_transport:=compressed`. |
| Bridge: `Bad frame size …` | Something other than the sender connected (an old bridge version without the handshake check). Update the code and rebuild. |
| Sender: cannot open camera | Try `--camera 1`. Close other apps using the camera (Teams, the Camera app). |
| Sender low fps | It forces MJPG already. Try `--width 960 --height 540`, and check that the room is bright, since webcams slow down in low light. |
| `usbipd` doesn't list the built-in camera | That's expected: it's MIPI, not USB. Use the Windows sender. |
| `position: null` / `no position` | Check `use_depth`, the `depth_topic` and `camera_info_topic` names, and that depth is aligned to the colour image. |
| Warning `No tf to map` | No TF from the camera to the map (localisation isn't running). The scanner reports in the camera frame meanwhile. |
| `build/`, `install/`, `log/` appear in `git status` | colcon was run from `src/`. Delete those three folders from `src/` and run colcon from `~/robot_challenge_workspace`. |
| The same code is reported only once | That's by design. Change `repeat_after_s` if you need more reports. |

## 12. Sharing changes (git)

Run these from `~/robot_challenge_workspace/src`:
```bash
git pull --rebase origin main          # get teammates' changes first
git add -A && git status               # review what will be committed
git commit -m "what you changed"
git push origin main
```
`.gitignore` keeps Python caches and colcon output out of the repo.

Anyone pulling new code must rebuild (section 2). If `requirements.txt` changed, they must also redo the pip installs in the [README, section 2](README.md#2-what-you-need-to-install-first): CPU torch first, then `requirements.txt`, then `pip3 uninstall -y setuptools`.
