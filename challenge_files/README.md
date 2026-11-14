# Challenge files: robot build of `team_perception`

This folder holds the code that runs during the challenge: the **QR scanner** and the
**human poster detector**. It is a slimmed copy of the `team_perception` package, with
the debug code taken out of the nodes and no test data or debugging tools.

It is split between two computers:

| Folder | Runs on | Nodes | Why there |
|---|---|---|---|
| `onboard/team_perception/` | the robot computer | `qr_scanner`, `poster_detector`, `image_throttle` | They need depth, TF and the map, which are too big to send over Wi-Fi. They're light enough for the robot's CPU. |
| `external/team_perception_external/` | a faster computer on the robot's network | `human_detector` (YOLO) | YOLO is the heaviest step (50–100 ms a frame on a CPU, much less on a GPU). It needs only a JPEG stream and sends back small JSON. |

```
 ROBOT (onboard)                                           EXTERNAL COMPUTER
 camera JPEG 30 Hz ─┬─► qr_scanner ─► /qr/detections, /found "QR found"
                    │
                    └─► image_throttle ─► /camera/color/image_offboard/compressed ─► human_detector
                         (10 Hz, ~6 Mbit/s)                   (over Wi-Fi)              (YOLO)
                                                                                          │
 depth + camera_info, TF, /map ─► poster_detector ◄────── /humans/detections ◄────────────┘
                                   │                       (JSON, a few KB/s)
                                   └─► /posters/detections, /found "Human found"
```

For a step-by-step breakdown of one frame's round trip and a setup and link-check guide for
both computers, see [onboard-external_communication.md](onboard-external_communication.md).

Raw depth, point clouds and lidar never leave the robot. Only about 6 Mbit/s of JPEG goes
out and a few KB/s of JSON comes back, instead of the ~370 Mbit/s that raw colour and depth
would need. Each `/humans/detections` message carries the camera stamp of its frame, so
`poster_detector` pairs it with the robot's depth image from the same moment, even if the
round trip takes a while (up to 2 s, the length of its depth buffer).

**Debugging tools, tests and sample media are on GitHub:**
**<https://github.com/ttticia/Robot-Dog-Grp4>** (folder `team_perception/`). That
version includes:

| Tool | What it's for |
|---|---|
| `/qr/debug_image`, `/humans/debug_image`, `/posters/debug_image` (and `/compressed`) | Live video with boxes and labels drawn, viewed in `rqt_image_view`. |
| `/posters/classifications` | Every person box with the edge-ring and map numbers behind its label, for tuning thresholds. |
| `human_detector log_every_s` | Saves annotated snapshots and CSV rows while people are in view. |
| `annotate` | Offline tool: runs the same detectors on photos and videos and writes annotated results to `test_runs/`. |
| `scripts/perception.sh` | All-in-one wrapper: setup, demos from video, viewers, bag checks. |
| `scripts/eval_qr_video.py`, `test/` | Decoder evaluation and unit tests, with the sample clip `QR.mp4` and test images. |

When you change the detection logic (`*_utils.py`) on GitHub, copy the change here too.
`qr_utils.py` (in both packages here) and `poster_utils.py` are identical to the GitHub
copies, and both versions must behave the same.

## Contents

```
challenge_files/
├── COLCON_IGNORE                 keeps the dev workspace from building these copies (see below)
├── onboard-external_communication.md  how the two halves talk, robot inputs, outputs (JSON),
│                                   integration, setup, link checks, debugging
├── onboard/team_perception/      ROS 2 package for the ROBOT
│   ├── data_required.md          checklist of the robot data the detections need
│   ├── install_deps.sh           installs every dependency (apt + pip)
│   ├── launch/robot.launch.py
│   ├── team_perception/
│   │   ├── qr_scanner_node.py       -> /qr/detections, /found "QR found"
│   │   ├── image_throttle_node.py   -> /camera/color/image_offboard/compressed (to external)
│   │   ├── poster_detector_node.py  -> /posters/detections, /found "Human found"
│   │   ├── qr_utils.py, poster_utils.py
│   └── package.xml, setup.py, setup.cfg, requirements.txt, LICENSE
└── external/team_perception_external/   ROS 2 package for the EXTERNAL computer
    ├── install_deps.sh           installs every dependency (apt + torch + pip + YOLO weights)
    ├── launch/external.launch.py
    ├── team_perception_external/
    │   ├── human_detector_node.py   -> /humans/detections (input to poster_detector)
    │   ├── human_utils.py, qr_utils.py
    └── package.xml, setup.py, setup.cfg, requirements.txt, LICENSE
└── nav_test/                     navigation (separate from perception)
    ├── example maze.step         CAD of an example maze (only cell size 1 m and wall height
    │                               ~45 cm are accurate)
    └── maze_explorer/            ROS 2 package: maze exploration with slam_toolbox + Nav2 +
                                    explore_lite, camera-coverage supervisor, Gazebo test
                                    world. Guide: nav_test/maze_explorer/README.md
```

The packages have different names, so both can be installed on the same computer (see
*Running without the external computer*).

## Why `COLCON_IGNORE`?

This folder lives in the dev workspace's `src/` next to the full `team_perception`
package. colcon refuses to build two packages with the same name, so the empty
`COLCON_IGNORE` file tells it to skip everything in here on the dev machine. **Don't copy
that file anywhere.** Copy only the inner package folders.

## Setup

Each package has an `install_deps.sh` that installs **all** its dependencies:

| | `onboard/team_perception/install_deps.sh` | `external/team_perception_external/install_deps.sh` |
|---|---|---|
| apt (needs sudo) | `ros-humble-cv-bridge`, `ros-humble-tf2-ros`, `ros-humble-tf2-geometry-msgs`, `ros-humble-launch-ros`, `python3-opencv`, `python3-numpy`, `python3-pyzbar`, `libzbar0`, `python3-pip` | the same, without `python3-pyzbar`/`libzbar0` |
| pip (`--user`) | `requirements.txt`: `zxing-cpp==2.3.0`, `numpy<2` | `torch==2.14.1` + `torchvision==0.29.2` (CPU build, or CUDA with `--gpu`), then `requirements.txt`: `ultralytics==8.4.210`, `numpy<2` |
| afterwards | checks that every module imports | removes pip `setuptools` (it breaks colcon on Humble), downloads `yolo11n.pt` to `~/.cache/team_perception/` and checks the imports |

Options: `--no-apt` skips the apt step (no sudo). The external script also takes `--gpu`. Both
scripts can be run again safely. The ROS 2 dependencies are also declared in each `package.xml`,
so `rosdep install --from-paths src --ignore-src -y` covers the apt part. The pip packages have
no rosdep keys, so they come only from the script.

### On the robot (ROS 2 Humble)

```bash
# 1. Copy the onboard package into the robot's workspace (from the dev machine)
scp -r challenge_files/onboard/team_perception <robot>:~/robot_ws/src/

# 2. On the robot: all dependencies (no torch or YOLO needed here)
bash ~/robot_ws/src/team_perception/install_deps.sh

# 3. Build (from the workspace root, never from src/)
cd ~/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select team_perception
source install/setup.bash
```

### On the external computer (Ubuntu 22.04, ROS 2 Humble)

```bash
# 1. Copy the external package into a workspace on that computer
mkdir -p ~/ext_ws/src && cp -r challenge_files/external/team_perception_external ~/ext_ws/src/

# 2. All dependencies, plus the YOLO weights (add --gpu on an NVIDIA machine)
bash ~/ext_ws/src/team_perception_external/install_deps.sh

# 3. Build
cd ~/ext_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select team_perception_external
source install/setup.bash
```

If the external computer has no internet, copy `~/.cache/team_perception/yolo11n.pt` (about
5 MB) from the dev machine to the same path. Without internet, the pip step also fails: install
the pip packages from wheels downloaded elsewhere (`pip3 download -d wheels ...`, then
`pip3 install --user --no-index --find-links wheels ...`).

### Network

The external computer must be on the robot's network and use the robot's ROS domain:

```bash
export ROS_DOMAIN_ID=4                       # the Lite3's domain
ros2 topic list | grep camera                # the robot's camera topics must show up
```

- **Use a wired link or 5 GHz Wi-Fi close to the access point.** The JPEG stream needs
  about 6 Mbit/s at 10 Hz.
- If topics don't show up, the network is probably blocking DDS multicast discovery. Set
  unicast peers in the DDS config, or bridge only the needed topics with
  `zenoh-bridge-ros2dds`.
- Check the link from the external computer:
  `ros2 topic hz /camera/color/image_offboard/compressed` should show about 10 Hz.
  If it is lower, or `/humans/detections` arrives more than 2 s late, posters can't be
  confirmed. Lower `offboard_rate_hz` on the robot.

## Running

Start the robot side first, then the external side. Either order works, but nothing
is detected as a poster until both are running.

```bash
# On the robot
ros2 launch team_perception robot.launch.py
ros2 launch team_perception robot.launch.py --show-args        # list all arguments
ros2 launch team_perception robot.launch.py offboard_rate_hz:=5.0   # slow Wi-Fi
ros2 launch team_perception robot.launch.py posters:=false     # QR scanner only

# On the external computer
ros2 launch team_perception_external external.launch.py
ros2 launch team_perception_external external.launch.py device:=0   # NVIDIA GPU
```

**Robot (`robot.launch.py`)**

| Launch argument | Default | Notes |
|---|---|---|
| `image_topic` | `/camera/color/image_raw` | Raw name; `/compressed` is added when `image_transport:=compressed`. |
| `image_transport` | `compressed` | `raw` or `compressed` (for `qr_scanner`). `image_throttle` always forwards `<image_topic>/compressed`. |
| `depth_topic` | `/camera/depth/image_raw` | The Lite3's depth is **not aligned** to colour; the nodes compensate (temporary fix, see onboard-external_communication.md §4). |
| `depth_camera_info_topic` | `/camera/depth/camera_info` | Depth intrinsics for that mapping. Set `''` if `depth_topic` is already aligned to colour. |
| `camera_info_topic` | `/camera/color/camera_info` | |
| `use_depth` | `true` | QR codes get map positions from depth. |
| `map_topic` | `/map` | Static map (transient local). |
| `map_frame` | `map` | Needs TF from the camera frame to this frame. |
| `map_mode` | `veto` | Poster map test: `require`, `veto` or `off`. |
| `offboard_topic` | `/camera/color/image_offboard/compressed` | Throttled JPEG stream for the external computer. |
| `offboard_rate_hz` | `10.0` | Frames per second sent to the external computer. |
| `qr`, `posters` | `true` | Turn the QR scanner, or the poster pipeline (`image_throttle` + `poster_detector`), on or off. |

**External computer (`external.launch.py`)**

| Launch argument | Default | Notes |
|---|---|---|
| `image_topic` | `/camera/color/image_offboard` | Raw name of the robot's throttled stream; `/compressed` is added. |
| `image_transport` | `compressed` | |
| `use_depth` | `false` | `true` streams raw depth (~150 Mbit/s) to fill in person positions. Wired link only; `poster_detector` doesn't need it. |
| `depth_topic`, `depth_camera_info_topic`, `camera_info_topic`, `map_frame` | as on the robot | Only used with `use_depth:=true`. |
| `model` | `yolo11n.pt` | Bare name → `~/.cache/team_perception/`, or a path. A GPU can afford a bigger model (e.g. `yolo11s.pt`). |
| `conf` | `0.4` | Minimum person confidence. |
| `imgsz` | `640` | YOLO input size. |
| `device` | `cpu` | `0` for the first CUDA GPU. |
| `rate_hz` | `10.0` | Max inferences per second. More than `offboard_rate_hz` gains nothing. |

Check the topic names with `ros2 topic list` before a run. Every node parameter can
still be set with `ros2 run <package> <node> --ros-args -p name:=value`. Parameter
meanings are documented in the GitHub README.

### Running without the external computer

If the external computer isn't available, install `team_perception_external` on the robot
as well (the external setup steps above, in the robot's workspace) and run YOLO there on the
full camera stream, at the cost of robot CPU:

```bash
ros2 launch team_perception robot.launch.py
ros2 launch team_perception_external external.launch.py image_topic:=/camera/color/image_raw \
    use_depth:=true rate_hz:=5.0
```

## Outputs

For the full message formats, what the robot must provide and how to connect other software to
these outputs, see [onboard-external_communication.md](onboard-external_communication.md).

| Topic | Type | From | Content |
|---|---|---|---|
| `/found` | `std_msgs/String` | robot | `QR found` once per new code, `Human found` once per new poster position. Other tools match these exact strings. |
| `/qr/detections` | `std_msgs/String` | robot | JSON per newly seen code: text, pixel, frame, position, snapshot path. |
| `/posters/detections` | `std_msgs/String` | robot | JSON per newly confirmed poster: id, position, votes, test reasons. |
| `/humans/detections` | `std_msgs/String` | external | JSON per frame with people: boxes, confidence. Real people **and** posters. Positions are `null` unless `use_depth:=true`. |
| `/camera/color/image_offboard/compressed` | `sensor_msgs/CompressedImage` | robot | Throttled camera JPEGs for the external computer. |

Records of each find are written on the robot, outside the workspace, and created only on
the first find. `~/qr_logs/` gets a CSV row plus a snapshot per code, and `~/poster_logs/`
gets a CSV row per confirmed poster.
