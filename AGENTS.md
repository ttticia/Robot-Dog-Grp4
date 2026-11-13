# Instructions for coding agents

This repo is the `src/` folder of a ROS 2 Humble colcon workspace at `~/robot_challenge_workspace`. Its job is perception for a robot-dog challenge: reading QR codes and detecting people, including pictures of people, from the robot's camera. The human-facing setup and run guide is [team_perception/README.md](team_perception/README.md), and the QR scanner's design notes are in [team_perception/QR_test_guide.md](team_perception/QR_test_guide.md). Read both before making non-trivial changes.

## Chat instructions
At the end of each prompt response, create summary in bullet points. Update this file when changes to the project are made.

If a process will run for more than 1 minute (a build, a full `annotate` run, a bag evaluation and so on), give the user a progress update at least every 2 minutes until it finishes: what is running, how far it has got (e.g. from `STATUS.txt` or the frame counter) and the expected time left.

## Layout

There is **one package**, `team_perception/`. The older `human_detection` and `qr_test_tools` packages were merged into it; don't recreate them.

| File in `team_perception/team_perception/` | Role |
|---|---|
| `qr_utils.py` | ROS-free QR logic: `decode_qr`, `draw_detections`, `_text_block`, `depth_at`, `deproject`. |
| `human_utils.py` | ROS-free person logic: `HumanDetector` (Ultralytics YOLO, class 0), `draw_humans`, `resolve_model`, `box_depth_pixel`. Reuses the drawing and depth helpers from `qr_utils`. |
| `qr_scanner_node.py` | Node `qr_scanner`: publishes `/qr/detections` and `/qr/debug_image[/compressed]`, logs to `~/qr_logs`. |
| `human_detector_node.py` | Node `human_detector`: publishes `/humans/detections` and `/humans/debug_image[/compressed]`, logs to `~/human_logs`. |
| `poster_utils.py` | ROS-free poster-vs-real logic: `edge_ring_test` (inverse-depth plane fit on the torso, extrapolated to a ring outside the box), `StaticMap` + `static_map_test`, `combine` (`map_mode` require/veto/off), `PosterTracker` (voting by position). |
| `poster_detector_node.py` | Node `poster_detector`: consumes `/humans/detections` (needs its `stamp` + `image_size`), the aligned depth, `camera_info`, `/map` (transient local) and TF. Publishes `/posters/detections` (confirmed only), `/posters/classifications` and `/posters/debug_image[/compressed]`. Logs to `~/poster_logs`. |
| `webcam_bridge_node.py` | Node `webcam_bridge`: TCP JPEG stream to `/camera/color/image_raw[/compressed]`. |
| `annotate.py` | Executable `annotate --type {qr,human,both} <inputs>`: the offline test tool. |

Also in the package: `scripts/` (`perception.sh`, the all-in-one command wrapper; `webcam_sender_windows.py`; `eval_qr_video.py`) and `test/` (unit tests; sample media in `test/testing_data/`: the clip `QR.mp4` and `test_images/`).

**Robot build: `challenge_files/`.** This is a slimmed copy of `team_perception` that runs aboard the dog: the three detector nodes, `qr_utils.py`, `human_utils.py` (without `draw_humans`), `poster_utils.py`, and `launch/robot.launch.py` (`ros2 launch team_perception robot.launch.py`). It has no debug topics, no `/posters/classifications`, no `log_every_s`, no `annotate`, no `webcam_bridge`, no scripts and no tests. `challenge_files/COLCON_IGNORE` keeps the dev build from seeing a duplicate package. Only the inner `team_perception/` folder goes onto the robot. `challenge_files/README.md` covers robot setup and links to the GitHub repo for the debugging tools. `challenge_files/team_perception/INTEGRATION.md` is for the people integrating the package into the robot: required inputs, every output topic with its JSON fields, and how to consume them. Rules:
- When a topic, JSON field, `/found` string or launch argument changes, update `INTEGRATION.md` in the same change.
- Keep `qr_utils.py` and `poster_utils.py` byte-identical to the dev copies, and when you change the detection logic or a node's behaviour, mirror it here.
- The dev package's flake8/pep257 tests don't cover this folder, so run `ament_flake8 --linelength 99` and `ament_pep257` in `challenge_files/team_perception` yourself.

Pure logic lives in the ROS-free `*_utils.py` modules, so it can be unit-tested without a ROS graph. Keep it that way: nodes and `annotate` are thin wrappers, and `annotate` must call the same functions as the nodes so that offline results match the robot.

## Build and test

```bash
cd ~/robot_challenge_workspace                    # NEVER run colcon from src/
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select team_perception
source install/setup.bash
cd src/team_perception && python3 -m pytest test/ -q   # includes flake8 + pep257 checks
```
- `test_annotate.py` uses a `FakeDetector`, so it runs fast. `test_human_utils.py` runs the real YOLO model once, on ultralytics' `bus.jpg`.
- Expected results: **1 xfailed** (`test_D.jpg`, a curved printout no decoder can read: leave it alone). There is no copyright-header test. The repo is Apache-2.0 (`LICENSE` at the root, and declared in `package.xml`/`setup.py`), and files don't carry per-file headers.
- Style: flake8 with **99-column lines** and pep257 docstrings. The style tests fail on *any* `.py` file inside a package folder, including scratch scripts, so don't leave throwaway scripts in `src/`.
- A colcon build doesn't delete executables that have been removed. Stale entries in `ros2 pkg executables` go away after `rm -rf build/<pkg> install/<pkg>` and a rebuild.

## Environment pitfalls (all hit on the dev machine)

- **numpy must stay below 2.** Humble's `cv_bridge` is built against numpy 1.x.
- **pip setuptools breaks colcon.** Installing torch drags in setuptools 78 and colcon fails with `_distribution_fullname`. Fix: `pip3 uninstall -y setuptools`, which restores the apt 59.6.
- **OpenCV:** ultralytics installs `opencv-python` 4.11 into `~/.local`, which shadows apt's 4.5.4. The QR tests pass with both versions. Never rely on `cv2.QRCodeDetector` for decoding: apt's 4.5.4 has no QUIRC. Decoding goes through zxing-cpp and pyzbar.
- **Videos must be H.264.** The pip OpenCV can't write `avc1`, and its `mp4v` fallback won't play in VS Code or browsers. `annotate.make_playable()` re-encodes with `imageio-ffmpeg`'s bundled ffmpeg; there is no system ffmpeg and no sudo. Any new code that writes video must do the same.
- **CPU only.** There is no GPU, and torch is the `+cpu` build. YOLO `yolo11n` takes about 50–100 ms per 640 px frame.
- **YOLO weights** go in `~/.cache/team_perception/`, through `human_utils.resolve_model`. Never let ultralytics download `*.pt` into the repo or the current folder.
- **rqt compressed images** need `ros-humble-image-transport-plugins`. If they're missing, view the raw topics (`/qr/debug_image`, `/humans/debug_image`).
- **`ros2 topic echo` failing with `!rclpy.ok()`** means the ROS daemon is stale: run `ros2 daemon stop; ros2 daemon start`.
- **Killing processes:** `pkill -f <pattern>` also matches your own shell when the pattern appears in the command line. Kill by PID from `pgrep -f` instead.
- **`ros2 run` is only a wrapper.** Killing its PID leaves the real node, a child under `install/.../lib/team_perception/`, running. Either start it with `setsid` and kill the whole group (`kill -INT -- -PGID`), as `perception.sh` does, or kill the child PIDs too. After any live test, check with `pgrep -fa "team_perception/lib|webcam_sender|topic echo|static_transform"` that nothing is left. A leftover `webcam_bridge` holds port 5005 and hijacks the next test.
- `sudo` needs a password, so you can't apt install. Ask the user to run the command.

## Running things for verification

- **Debugging tools are opt-in.** Don't make anything debug-only run at startup. Debug images are drawn only while subscribed; `human_detector log_every_s` defaults to 0; `poster_detector publish_debug` defaults to false; `/posters/classifications` is only published while subscribed; log files are created on the first find. `perception.sh demo/robot` open rqt and the `/found` listener only with `--view`/`--echo`.
- **Prefer `team_perception/scripts/perception.sh`** (`help` lists the commands). It sources ROS and the workspace itself; `demo <video> qr|human|both` runs the full live pipeline and cleans up on Ctrl+C or exit. Keep it updated when nodes, topics or setup steps change.

- **Live pipeline from a video, without a camera:** start these four, each in the background:
  1. `ros2 run team_perception webcam_bridge`
  2. the node, with `--ros-args -p image_transport:=compressed -p use_depth:=false`
  3. `python3 team_perception/scripts/webcam_sender_windows.py --video team_perception/test/testing_data/QR.mp4 --loop`
  4. `rqt_image_view <debug topic>`

  Stop them all when you're done.
- **Offline (preferred for checking results):** `ros2 run team_perception annotate --type {qr,human,both} <inputs>` writes to `team_perception/test_runs/<type>/<timestamp>_<name>/`. Each run folder holds `STATUS.txt` (RUNNING/DONE/FAILED/INTERRUPTED + progress), `input/` (copied media), the annotated video and/or `photos/`, `detections.csv` and `summary.txt`/`summary.json`. To check a background run, read its `STATUS.txt`; don't poll the video. Videos are written as `*.partial.mp4` and renamed when complete. Keep the layout of qr/, human/ and both/ run folders, each with a copy of its input.
  - A full run on `QR.mp4` (4,112 frames, 640×480) takes about 4 minutes with `--type qr` and about 10 minutes with `human` or `both`, so run it in the background. Use `--max-frames N` for quick checks.
- **Known human results on QR.mp4:** a poster of a person next to the QR code at about 70–72 s (confidence about 0.85), and real people in the lab background at 4–14 s and 55–100 s. No false positives were seen on the toys or boards.
- **Known QR.mp4 results:** codes `J` (about 32 s), `I` (35–73 s) and the "rescue team needs medical supplies…" message (about 70 s). A regression in the QR decoder shows up as one of these going missing.
- **poster_detector:** `QR.mp4` has no depth, so it can't be tested on that clip. `test_poster_utils.py` covers the logic on synthetic depth and maps. For a ROS smoke test, publish fake depth, `camera_info`, `/map`, a static TF `map`→camera and `/humans/detections` with matching stamps. The thresholds are still untuned on real robot data.
- **`/found` (std_msgs/String):** `qr_scanner` publishes `QR found` once per new code text, and `poster_detector` publishes `Human found` once per new poster position. Both are also logged. Keep these exact strings, because other tools may match on them.
- **Runtime logs:** `~/qr_logs/`, `~/human_logs/` and `~/poster_logs/`, outside the repo.

## Repo rules

- `test_runs/`, model weights (`*.pt`, `*.onnx`), caches and colcon output are gitignored. Don't commit them, and don't commit large media other than deliberate test fixtures.
- Commit or push only when the user asks. The remote is `origin main`. Pull with `--rebase` first.
- **New pip dependencies** go in that package's `requirements.txt`, with an exact version pin, plus a note in the README's install section.
- **New parameters or topics:** update the README tables, and the QR guide for scanner changes, in the same change.
- **Moving or deleting tracked files:** use `git mv` / `git rm`, so history is kept.
