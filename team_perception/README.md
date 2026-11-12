# Robot-Dog-Grp4: perception (`team_perception`)

## Quick start: `perception.sh` (no `source` needed)

[`scripts/perception.sh`](scripts/perception.sh) wraps every setup and detection command. It sources ROS 2 and the workspace by itself, so you can run it from any terminal. Optionally, add an alias once:
```bash
echo "alias perception='$HOME/robot_challenge_workspace/src/team_perception/scripts/perception.sh'" >> ~/.bashrc && source ~/.bashrc
```

| Command | What it does |
|---|---|
| `perception setup` | Installs CPU torch and `requirements.txt` with pip, removes pip's setuptools, builds and checks. Prints the `sudo apt install ...` line if any system packages are missing. |
| `perception build [--clean]` | Rebuilds from the workspace root. Run it after every pull or code change. |
| `perception check` / `perception test` | Shows decoder, YOLO, numpy and torch versions and the executables; runs the unit and style tests. |
| `perception demo <video> [qr\|human\|both] [--loop] [--view] [--echo]` | The whole live pipeline from a video file: bridge, nodes and video. The video waits until the nodes are ready. `--view` opens rqt windows and `--echo` prints `/found`; both are off by default. **Ctrl+C stops everything.** |
| `perception qr` / `human` / `poster` `[ros args]` | One node on the robot's camera topics, e.g. `perception qr -p use_depth:=false`. |
| `perception robot [qr\|human\|poster\|all] [--echo]` | Several nodes together on the robot; Ctrl+C stops all of them. `--echo` also prints `/found`. |
| `perception bridge` / `perception play <video> [--loop]` | Webcam/video bridge, and a video sender, for building a pipeline by hand. |
| `perception view <qr\|human\|poster>` / `perception echo <found\|qr\|human\|poster>` | Opens rqt on a debug image (falls back to raw if the compressed plugin is missing), or prints a topic. |
| `perception annotate -t <qr\|human\|both> <inputs...>` | Offline annotation into `test_runs/` (section 7). Relative paths work from any folder. |
| `perception bag-check <bag folder>` | Shows `ros2 bag info` and whether the database is actually readable (catches unfinished copies). |

**Robot camera topics** default to the Orbbec camera recorded in the `test_run` bag: `/camera/color/image_raw` (compressed), `/camera/depth/image_raw` and `/camera/color/camera_info`. Override them per run with environment variables, e.g. `DEPTH_TOPIC=/camera/aligned_depth/image_raw perception poster`. The other variables are `IMAGE_TOPIC`, `IMAGE_TRANSPORT`, `CAMERA_INFO_TOPIC` and `MAP_TOPIC`. `perception help` lists everything.

The rest of this README shows the underlying commands, for when you need finer control.

## Rebuild (do this after every pull or code change)

`perception build` does all of this. By hand:
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
| `poster_detector` | **Poster node.** It decides which of `human_detector`'s people are posters, using depth (the edge-ring test) and the static map. It publishes only posters (see section 6). |
| `webcam_bridge` | Feeds a Windows laptop webcam, or a video file, into ROS as a camera topic. |
| `annotate` | **Offline test tool.** It annotates photos and videos with QR, person, or both kinds of detection. Each run is saved in `test_runs/<qr\|human\|both>/` (see [section 7](#7-offline-testing-annotate-photos-and-videos)). |

| File | Contents |
|---|---|
| `team_perception/qr_utils.py`, `human_utils.py`, `poster_utils.py` | Detection and drawing logic with no ROS imports, so it is unit-tested without ROS. Both the nodes and `annotate` use it. |
| `team_perception/*_node.py`, `annotate.py` | The executables above. |
| `scripts/` | `webcam_sender_windows.py` (camera/video sender) and `eval_qr_video.py` (QR decoder benchmark). |
| `test/` | Unit tests. Sample media is in `test/testing_data/`: the clip `QR.mp4` and the photos in `test_images/`. |
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
    --video ~/robot_challenge_workspace/src/team_perception/test/testing_data/QR.mp4 --loop
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
Each new code is also recorded in `~/qr_logs/`, as a CSV row plus an annotated snapshot. The file is created on the first code, so startup writes nothing. The debug images are only drawn while something (e.g. rqt) subscribes.

**First-sighting announcements:** the first time each new code is seen, `qr_scanner` logs `QR found` and publishes it on `/found`. When each new poster is confirmed for the first time, `poster_detector` does the same with `Human found`. Repeat reports don't announce again. To watch both at once:
```bash
ros2 topic echo /found std_msgs/msg/String --field data     # prints "QR found" / "Human found"
``` For parameters, the JSON format and common problems, see the [QR test guide](QR_test_guide.md).

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
- **Debugging only:** with `-p log_every_s:=5`, an annotated snapshot and a CSV row go to `~/human_logs/` at most every 5 s. It's off (0) by default, so no files are written.

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
| `log_dir` / `log_every_s` | `~/human_logs` / 0 (off) | Debugging snapshots and CSV: where, and how often. |

### Posters only: `poster_detector`

YOLO can't tell a poster of a person from a real person. `poster_detector` takes `human_detector`'s boxes and uses depth and the static map to decide which ones are **posters**. It publishes only those.

```
human_detector ──/humans/detections──┐
aligned depth + camera_info ─────────┼─► poster_detector ─► /posters/detections   (confirmed posters, once each)
/map + TF camera→map ────────────────┘                    ├► /posters/classifications (every box + the numbers behind its label)
                                                          └► /posters/debug_image[/compressed] (magenta poster / blue real / grey unknown)
```
1. **Edge-ring test (depth):** it fits the surface of the torso region as a plane, then looks at a thin ring just outside the box (left, right and top).
   - **Poster:** the ring lies on the same plane, because the surface continues past the print. Tilted posters work too.
   - **Real person:** the ring is at least 0.25 m *behind* the plane, because they stand in front of their background.
2. **Static-map test:** the torso point is transformed into the `map` frame.
   - **Poster:** within 0.3 m of an occupied map cell, i.e. on a wall or board.
   - **Real person:** no occupied cell within 0.6 m, i.e. in free space.
3. **Combining (`map_mode`):**
   - A ring "real" always wins.
   - `veto` (the default): the map can only overrule a poster verdict by finding free space.
   - `require`: the map must also say "on a wall".
   - `off`: the ring test alone.
4. **Voting:** each poster is tracked by map position. It is reported once it has `vote_need` (5) poster votes among its last `vote_window` (8) frames, then again only after `repeat_after_s`.

```bash
# needs human_detector running, depth aligned to colour, localisation (map->camera TF) and /map
ros2 run team_perception poster_detector --ros-args \
  -p depth_topic:=/camera/aligned_depth_to_color/image_raw -p camera_info_topic:=/camera/color/camera_info \
  -p image_topic:=/camera/color/image_raw -p image_transport:=compressed
ros2 topic echo /posters/detections std_msgs/msg/String
ros2 run rqt_image_view rqt_image_view /posters/debug_image/compressed
```
The first confirmation of each new poster also prints `Human found` and publishes it on `/found` (see section 5). Posters are matched by map position, so a poster seen again later isn't announced twice.

Each `/posters/detections` message is JSON: `{"stamp", "id", "frame", "position": [x,y,z], "votes": "6/8", "conf", "box", "ring", "map", "snapshot"}`. A CSV row per confirmed poster goes to `~/poster_logs/`, plus a snapshot when `publish_debug` is on. `/posters/classifications` is only filled while something listens to it.

| Parameter | Default | Notes |
|---|---|---|
| `detections_topic` | `/humans/detections` | Output of `human_detector`. |
| `depth_topic`, `camera_info_topic` | aligned depth, colour info | Depth **must be aligned to the colour image**. |
| `publish_debug`, `image_topic`, `image_transport` | **false**, colour, `raw` | Debugging: subscribe to the colour stream for `/posters/debug_image` and per-poster snapshots. Off by default, because it costs a full colour stream. |
| `map_topic`, `map_frame`, `use_map`, `map_mode` | `/map`, `map`, true, `veto` | `veto` / `require` / `off` (see above). |
| `max_dt_s` | 0.1 | Maximum time difference between a detection and the depth frame used. |
| `ring_tol_m`, `ring_tol_quad` | 0.05, 0.01 | "On the plane" tolerance = `ring_tol_m + ring_tol_quad × z²`. |
| `ring_step_m` | 0.25 | Minimum distance the ring must be behind the plane to count as a real person. |
| `ring_frac`, `min_sides`, `max_range_m` | 0.10, 2, 4.0 | Ring thickness (fraction of box width), usable sides needed, and the range beyond which the result is unknown. |
| `map_on_radius_m`, `map_free_radius_m`, `map_occupied` | 0.30, 0.60, 65 | Map-test distances and the occupied threshold. |
| `merge_radius_m`, `vote_window`, `vote_need`, `repeat_after_s` | 0.5, 8, 5, 60 | Tracking and voting. |

**Limits:**
- **Person against a wall:** someone standing within about 25 cm of a wall comes out as *unknown*, not real or poster.
- **Posters need a backing surface:** the print must have a margin or board around the figure. If the box reaches the board's edge, the ring falls off the board.
- **The static map must be built with no people in it, and with the posters' walls or boards in it.** A live SLAM map (mapping mode) maps people who stand still. Under `require`, a poster on a board added after mapping is never confirmed.
- **Untested on real data:** so far it has only been run on synthetic scenes (unit tests and a fake ROS scene). The thresholds still need tuning on a rosbag recorded on the robot.

## 7. Offline testing: annotate photos and videos

`annotate` runs the same detection code as the nodes on files, with no ROS graph needed. `--type` chooses the test:

```bash
cd ~/robot_challenge_workspace/src
ros2 run team_perception annotate --type qr    team_perception/test/testing_data/QR.mp4
ros2 run team_perception annotate --type human photo1.jpg photos_folder/ --conf 0.3
ros2 run team_perception annotate --type both  team_perception/test/testing_data/QR.mp4 team_perception/test/testing_data/test_images --name lab_walk
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
