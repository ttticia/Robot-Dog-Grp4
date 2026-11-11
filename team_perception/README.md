# Robot-Dog-Grp4: perception (`team_perception`)

## Rebuild (do this after every pull or code change)

```bash
cd ~/robot_challenge_workspace          # always build from the workspace, never from src/
source /opt/ros/humble/setup.bash
colcon build --symlink-install          # or: colcon build --packages-select team_perception
source install/setup.bash               # in every terminal you'll run nodes from
```
- If `team_perception/requirements.txt` changed in the pull, re-run the pip installs from [section 2](#2-what-you-need-to-install-first) first.
- After an **executable is added or removed**, open a new terminal or re-run `source install/setup.bash`, so that `ros2 run` can find it. A removed executable lingers until you run `rm -rf build/team_perception install/team_perception` and rebuild.
- If the build fails inside `setuptools` with `_distribution_fullname` in the traceback, a pip install has upgraded setuptools. Fix it with `pip3 uninstall -y setuptools`, which restores the Ubuntu copy that colcon expects.

---

## 1. What's in this package

This repo is the workspace's `src/` folder. It holds one ROS 2 package, `team_perception`, which has two detector nodes and an offline test tool:

| Executable | What it does |
|---|---|
| `qr_scanner` | **QR code node.** It reads codes from the camera and publishes the text and the code's map position. |
| `human_detector` | **Person node** (Ultralytics YOLO). It finds people in camera images, both real people and *pictures* of people such as posters and printouts. It publishes boxes and map positions. |
| `webcam_bridge` | Feeds a Windows laptop webcam, or a video file, into ROS as a camera topic. |
| `annotate` | **Offline test tool.** It annotates photos and videos with QR, person, or both kinds of detection. Each run is saved in `test_runs/<qr\|human\|both>/` (see [section 7](#7-offline-testing-annotate-photos-and-videos)). |

| File | Contents |
|---|---|
| `team_perception/qr_utils.py`, `human_utils.py` | Detection and drawing logic with no ROS imports, so it is unit-tested without ROS. Both the nodes and `annotate` use it. |
| `team_perception/*_node.py`, `annotate.py` | The executables above. |
| `scripts/` | `webcam_sender_windows.py` (camera/video sender) and `eval_qr_video.py` (QR decoder benchmark). |
| `test/` | Unit tests, the sample clip `QR.mp4` and the sample photos in `test_images/`. |
| [QR_test_guide.md](QR_test_guide.md) | The QR scanner in depth: how it works, its parameters, video testing and troubleshooting. |

## 2. What you need to install first

**Operating system:**
- Ubuntu 22.04 with **ROS 2 Humble**. It can be native Linux, or WSL2 on Windows 11.
- On WSL2, install Humble inside Ubuntu by following the [official instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) (`ros-humble-desktop`).

**System packages:**
```bash
sudo apt update
sudo apt install python3-pip python3-colcon-common-extensions \
                 python3-pyzbar python3-qrcode \
                 ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
                 ros-humble-image-transport-plugins ros-humble-rqt-image-view
```
`ros-humble-image-transport-plugins` lets rqt show the `.../compressed` debug topics. Without it, rqt says *"loading image transport plugin failed"*. As a workaround, view the raw topic instead (`/qr/debug_image`, `/humans/debug_image`).

**Python packages from pip** (run these after cloning, in step 3):
```bash
cd ~/robot_challenge_workspace
pip3 install --user torch==2.14.1 torchvision==0.29.2 --index-url https://download.pytorch.org/whl/cpu
pip3 install --user -r src/team_perception/requirements.txt    # zxing-cpp, ultralytics, numpy<2, imageio-ffmpeg
pip3 uninstall -y setuptools      # torch installs a setuptools that breaks colcon on Humble
```
- **Why two QR decoders?** The OpenCV that ships with Humble (4.5.4) **cannot decode QR codes**. We use **zxing-cpp** as the main decoder because it's fast and reads angled codes, and **pyzbar** as a backup.
- **numpy must stay below 2.** ROS Humble's `cv_bridge` is built against numpy 1.x.
- **CPU torch** is about 200 MB. On a machine with an NVIDIA GPU, leave out `--index-url ...` to get the CUDA build, then run with `-p device:=0`.
- **Model weights:** the YOLO weights (`yolo11n.pt`, about 5 MB) download automatically on first use to `~/.cache/team_perception/`. The robot needs internet access once, or copy the file there.

**To test with a laptop webcam on Windows**, also install this in Windows PowerShell (not WSL):
```powershell
pip install opencv-python numpy
```

## 3. Get the code and build

This repo **is** the workspace's `src/` folder, so clone it as `src`:
```bash
mkdir -p ~/robot_challenge_workspace && cd ~/robot_challenge_workspace
git clone git@github.com:ttticia/Robot-Dog-Grp4.git src
# ... pip installs from section 2 ...
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
If you'd rather not run the `source` lines in every new terminal, add them to `~/.bashrc`:
```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/robot_challenge_workspace/install/setup.bash' >> ~/.bashrc
```

**Check the install:**
```bash
python3 -c "from team_perception import qr_utils; print(qr_utils.BACKENDS)"   # should print: zxing-cpp + pyzbar
python3 -c "import ultralytics, torch, numpy; print(ultralytics.__version__, torch.__version__, numpy.__version__)"
cd ~/robot_challenge_workspace/src/team_perception && python3 -m pytest test/ -q
```
The pytest run should end with no failures and **1 xfailed**. That one is an expected failure: `test_D.jpg`, a curved printout that no decoder can read. The first run also downloads the YOLO weights.

## 4. Camera input

Both nodes subscribe to the same camera topic, so they can run side by side.

### On the robot
Find the camera topic names with `ros2 topic list | grep -i -e image -e camera_info -e depth`. Then pass them to each node, as shown in sections 5 and 6.

### With a laptop webcam (WSL)
WSL can't see a built-in laptop camera directly. A script on Windows streams the camera into ROS instead. Each step below runs in its own terminal:
```bash
# WSL: receive the stream; it publishes /camera/color/image_raw[/compressed]
ros2 run team_perception webcam_bridge
```
```powershell
# Windows PowerShell: send the camera (copy the script from team_perception/scripts/ to e.g. C:\qr\ first)
python C:\qr\webcam_sender_windows.py
```

### With a video file
This uses the same bridge. In place of the Windows sender, play the video from WSL:
```bash
python3 ~/robot_challenge_workspace/src/team_perception/scripts/webcam_sender_windows.py \
    --video ~/robot_challenge_workspace/src/team_perception/test/QR.mp4 --loop
```

## 5. QR node (`qr_scanner`)

```bash
# Robot (with depth, which gives map positions):
ros2 run team_perception qr_scanner --ros-args \
  -p image_topic:=/camera/color/image_raw -p image_transport:=compressed \
  -p depth_topic:=/camera/aligned_depth_to_color/image_raw \
  -p camera_info_topic:=/camera/color/camera_info
# Laptop webcam / video (no depth):
ros2 run team_perception qr_scanner --ros-args -p image_transport:=compressed -p use_depth:=false
```
To see the results:
```bash
ros2 run rqt_image_view rqt_image_view /qr/debug_image/compressed   # live video with green boxes + decoded text
ros2 topic echo /qr/detections std_msgs/msg/String                  # one JSON message per new code
```
Each code is also logged to `~/qr_logs/`, as a CSV row plus an annotated snapshot. For parameters, the JSON format and common problems, see the [QR test guide](QR_test_guide.md).

## 6. Human node (`human_detector`)

```bash
# Robot (with depth, which gives map positions):
ros2 run team_perception human_detector --ros-args \
  -p image_topic:=/camera/color/image_raw -p image_transport:=compressed \
  -p depth_topic:=/camera/aligned_depth_to_color/image_raw \
  -p camera_info_topic:=/camera/color/camera_info
# Laptop webcam / video (no depth):
ros2 run team_perception human_detector --ros-args -p image_transport:=compressed -p use_depth:=false
```
To see the results:
```bash
ros2 run rqt_image_view rqt_image_view /humans/debug_image/compressed   # live video with orange person boxes
ros2 topic echo /humans/detections std_msgs/msg/String                  # JSON per inference with >= 1 person
```
Each `/humans/detections` message is a JSON object: `{"stamp", "count", "snapshot", "people": [{"box": [x1,y1,x2,y2], "conf", "pixel", "frame", "position"}]}`.
- `position` is in the map frame, or the camera frame if there is no TF. It is `null` without depth.
- The depth is sampled at the torso, which is the box centre, 40% of the way down.
- An annotated snapshot and a CSV row go to `~/human_logs/` at most every `log_every_s`.

YOLO runs on a background thread on the newest frame, so the debug video stays at full frame rate even when inference is slower. On the dev laptop's CPU, `yolo11n` manages about 7 inferences per second at 640 px.

| Parameter | Default | Notes |
|---|---|---|
| `image_topic` / `image_transport` | `/camera/color/image_raw` / `raw` | Use `compressed` whenever you can. |
| `depth_topic`, `camera_info_topic`, `use_depth`, `map_frame` | as for `qr_scanner` | Depth must be aligned to the colour image. |
| `model` | `yolo11n.pt` | A bare name loads from `~/.cache/team_perception/`; a path is used as given. Bigger models (`yolo11s.pt`) are more accurate and slower. |
| `conf` | 0.4 | Minimum confidence. Lower it if posters or distant people are missed. |
| `imgsz` | 640 | YOLO input size. Smaller is faster but misses small or far people. |
| `device` | `cpu` | Use `0` for the first CUDA GPU. |
| `rate_hz` | 10 | Maximum inferences per second (0 = as fast as possible). |
| `hold_s` | 0.5 | How long boxes stay drawn after the last inference. |
| `publish_debug` | true | Publishes the debug images (only while something subscribes). |
| `log_dir` / `log_every_s` | `~/human_logs` / 5.0 | Where snapshots and the CSV go, and how often. |

## 7. Offline testing: annotate photos and videos

`annotate` runs the same detection code as the nodes on files, with no ROS graph needed. `--type` chooses the test:

```bash
cd ~/robot_challenge_workspace/src
ros2 run team_perception annotate --type qr    team_perception/test/QR.mp4
ros2 run team_perception annotate --type human photo1.jpg photos_folder/ --conf 0.3
ros2 run team_perception annotate --type both  team_perception/test/QR.mp4 team_perception/test/test_images --name lab_walk
ros2 run team_perception annotate --type qr    /mnt/c/qr_clip --frames-as-video --fps 30   # recorded frame folder
```
The inputs can be any mix of video files, photos and folders of either. Each call creates **one run folder**, sorted by test type:
```
team_perception/test_runs/
  qr/     <date>_<time>_<name>/
  human/  <date>_<time>_<name>/
  both/   <date>_<time>_<name>/
      STATUS.txt             RUNNING / DONE / FAILED / INTERRUPTED, progress + ETA, finished steps
      input/                 copies of the input media (skip with --no-copy)
      annotated_<video>.mp4  boxes, labels and a status bar (time, ms per detector, counts)
      photos/                annotated photos ("NOTHING FOUND" when empty)
      first_seen/            QR only: the frame where each code was first read
      detections.csv         one row per detection: input, frame, time, kind (qr/person), label, conf, ms, points
      summary.txt / .json    hit rates, timings, QR codes seen and when, people counts, exact command + settings
```
**Status while it runs:**
- **In the terminal:** each step is printed (copying inputs, loading YOLO, input *i*/*N*, re-encoding, summary). During a video, a live line shows frames done, %, speed, ETA, and the QR codes and people found so far. When output is redirected to a file, that line is printed every 10 s instead.
- **In the run folder:** `STATUS.txt` shows the same information, so you can check a background run from VS Code. It ends in `DONE`, `FAILED` (with the error) or `INTERRUPTED` (Ctrl+C).
- **Unfinished videos:** a video still being written is named `annotated_<name>.partial.mp4`. It is renamed when it's complete and playable. A `.partial.mp4` left behind means the run didn't finish.

Annotated videos are always H.264, so they play in VS Code and browsers. The pip OpenCV can't write H.264, so `annotate` re-encodes its output with `imageio-ffmpeg` (from `requirements.txt`).

Videos copy the nodes' behaviour:
- QR: every frame gets a fast pass, and every 5th frame gets a full scan (`--full-scan-every`).
- YOLO runs on every frame by default. Use `--every N` to run it on every Nth frame.
- Boxes stay drawn for `--hold-s` (0.5 s).
- Inputs larger than 1280×720 are scaled down, as the webcam sender does (`--max-width 0` turns this off).

Unlike the live nodes, every frame is processed and none are dropped. Use `--max-frames N` for a quick partial run, `--no-video` for statistics only, and `--help` for everything else.

On this CPU, a full `--type both` run on `QR.mp4` (2 minutes 17 seconds, 640×480) takes about 10 minutes, so start long runs in the background. `test_runs/` is ignored by git, because the annotated videos are large.

For QR benchmarking per decoding strategy, use `python3 team_perception/scripts/eval_qr_video.py <clip>` (see the QR guide, section 9).

## 8. Working on the repo

```bash
cd ~/robot_challenge_workspace/src
git pull --rebase origin main      # get everyone else's changes first
# ... make changes, rebuild (top of this file), run the tests ...
git add -A && git status           # check what you're committing
git commit -m "what you changed"
git push origin main
```
- **Code style:** keep the code passing the flake8 (99-column) and pep257 checks, which `python3 -m pytest test/` and `colcon test --packages-select team_perception` both run.
- **Never commit** `build/`, `install/`, `log/`, `__pycache__/`, `test_runs/` or model weights (`*.pt`). `.gitignore` already excludes them.
- **AI coding agents** (Claude Code, Codex, Copilot and others) follow [`../AGENTS.md`](../AGENTS.md). Keep it up to date when the build, layout or pitfalls change.
