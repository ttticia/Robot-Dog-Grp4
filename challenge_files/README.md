# Challenge files: robot build of `team_perception`

This folder holds the code that runs aboard the robot dog: the **QR scanner** and the
**human poster detector**. It is a slimmed copy of the `team_perception` package, with
the debug code taken out of the nodes and no test data or debugging tools.

**Debugging tools, tests and sample media are on GitHub:**
**<https://github.com/ttticia/Robot-Dog-Grp4>** (folder `team_perception/`). That
version includes:

| Tool | What it's for |
|---|---|
| `/qr/debug_image`, `/humans/debug_image`, `/posters/debug_image` (and `/compressed`) | Live video with boxes and labels drawn, viewed in `rqt_image_view`. |
| `/posters/classifications` | Every person box with the edge-ring and map numbers behind its label, for tuning thresholds. |
| `human_detector log_every_s` | Saves annotated snapshots and CSV rows while people are in view. |
| `annotate` | Offline tool: runs the same detectors on photos and videos and writes annotated results to `test_runs/`. |
| `webcam_bridge` + `scripts/webcam_sender_windows.py` | Feeds a laptop webcam or video file into ROS, so the pipeline can run without the robot. |
| `scripts/perception.sh` | All-in-one wrapper: setup, demos from video, viewers, bag checks. |
| `scripts/eval_qr_video.py`, `test/` | Decoder evaluation and unit tests, with the sample clip `QR.mp4` and test images. |

When you change the detection logic (`*_utils.py`) on GitHub, copy the change here too.
The two copies of `qr_utils.py` and `poster_utils.py` are identical, and both packages
must behave the same on the robot.

## Contents

```
challenge_files/
├── COLCON_IGNORE            keeps the dev workspace from building this copy (see below)
└── team_perception/         the ROS 2 package to put on the robot
    ├── INTEGRATION.md       topics, message formats, how to integrate on the robot
    ├── launch/robot.launch.py
    ├── team_perception/
    │   ├── qr_scanner_node.py       -> /qr/detections, /found "QR found"
    │   ├── human_detector_node.py   -> /humans/detections (input to poster_detector)
    │   ├── poster_detector_node.py  -> /posters/detections, /found "Human found"
    │   ├── qr_utils.py, human_utils.py, poster_utils.py
    ├── package.xml, setup.py, setup.cfg, requirements.txt, LICENSE
```

`poster_detector` judges the person boxes that `human_detector` finds, so the poster
pipeline needs both nodes. The launch file starts all three.

## Why `COLCON_IGNORE`?

This folder lives in the dev workspace's `src/` next to the full `team_perception`
package. colcon refuses to build two packages with the same name, so the empty
`COLCON_IGNORE` file tells it to skip this copy on the dev machine. **Don't copy that
file onto the robot.** Copy only the inner `team_perception/` folder.

## Setup on the robot (ROS 2 Humble)

```bash
# 1. Copy the package into the robot's workspace (from the dev machine)
scp -r challenge_files/team_perception <robot>:~/robot_ws/src/

# 2. On the robot: system and pip dependencies
sudo apt install python3-pyzbar ros-humble-cv-bridge ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs
pip3 install --user torch==2.14.1 torchvision==0.29.2 \
    --index-url https://download.pytorch.org/whl/cpu
pip3 install --user -r ~/robot_ws/src/team_perception/requirements.txt
pip3 uninstall -y setuptools     # torch's setuptools breaks colcon on Humble

# 3. Build (from the workspace root, never from src/)
cd ~/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select team_perception
source install/setup.bash
```

The first time `human_detector` starts, it downloads the YOLO weights (`yolo11n.pt`,
about 5 MB) to `~/.cache/team_perception/`. If the robot has no internet, copy that file
from the dev machine to the same path first.

## Running

```bash
ros2 launch team_perception robot.launch.py
ros2 launch team_perception robot.launch.py --show-args        # list all arguments
ros2 launch team_perception robot.launch.py depth_topic:=/camera/aligned_depth/image_raw
ros2 launch team_perception robot.launch.py posters:=false     # QR scanner only
ros2 launch team_perception robot.launch.py qr:=false          # poster pipeline only
```

| Launch argument | Default | Notes |
|---|---|---|
| `image_topic` | `/camera/color/image_raw` | Raw name; `/compressed` is added when `image_transport:=compressed`. |
| `image_transport` | `compressed` | `raw` or `compressed`. |
| `depth_topic` | `/camera/depth/image_raw` | Must be **aligned to the colour image** for the poster detector. |
| `camera_info_topic` | `/camera/color/camera_info` | |
| `use_depth` | `true` | QR codes and people get map positions from depth. |
| `map_topic` | `/map` | Static map (transient local). |
| `map_frame` | `map` | Needs TF from the camera frame to this frame. |
| `map_mode` | `veto` | Poster map test: `require`, `veto` or `off`. |
| `qr`, `posters` | `true` | Turn the QR scanner or the poster pipeline on or off. |

Check the topic names with `ros2 topic list` before a run. Every node parameter can
still be set with `ros2 run team_perception <node> --ros-args -p name:=value`. Parameter
meanings are documented in the GitHub README.

## Outputs

For the full message formats, what the robot must provide and how to connect other software to
these outputs, see [team_perception/INTEGRATION.md](team_perception/INTEGRATION.md).


| Topic | Type | Content |
|---|---|---|
| `/found` | `std_msgs/String` | `QR found` once per new code, `Human found` once per new poster position. Other tools match these exact strings. |
| `/qr/detections` | `std_msgs/String` | JSON per newly seen code: text, pixel, frame, position, snapshot path. |
| `/humans/detections` | `std_msgs/String` | JSON per frame with people: boxes, confidence, positions. Real people **and** posters. |
| `/posters/detections` | `std_msgs/String` | JSON per newly confirmed poster: id, position, votes, test reasons. |

Records of each find are written outside the workspace and created only on the first
find. `~/qr_logs/` gets a CSV row plus a snapshot per code, and `~/poster_logs/` gets a
CSV row per confirmed poster.
