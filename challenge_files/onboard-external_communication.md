# Onboard ↔ external communication and robot integration

How the `onboard` package (`team_perception`, on the robot) and the `external` package
(`team_perception_external`, on a faster computer) split the perception work and talk to each
other over ROS 2:
- what the robot must provide;
- what each detector publishes, and how the robot's other software should use it;
- setting up both computers, checking the link, and debugging a recorded run (including `/map` and `/tf`).

The one-page checklist of the robot data the detections need is
[onboard/team_perception/data_required.md](onboard/team_perception/data_required.md). All launch
arguments are listed in [README.md](README.md#running).

## In short

```bash
ros2 launch team_perception robot.launch.py                       # on the robot
ros2 launch team_perception_external external.launch.py           # on the external computer
```

- The robot runs `qr_scanner`, `poster_detector` and `image_throttle`.
- The external computer, on the robot's network with the same `ROS_DOMAIN_ID`, runs the YOLO
  `human_detector` on a 10 Hz JPEG stream from the robot.
- Depth, TF and the map stay on the robot.
- **Without the external computer, QR codes still work but no poster is ever confirmed.**
- To know when the robot has found something, listen to **`/found`**: `QR found` or
  `Human found`, once per new find.
- For *what* was found and *where*, read `/qr/detections` and `/posters/detections` (JSON in a
  `std_msgs/String`).

## 1. Why the work is split

| | Robot computer (`onboard/team_perception`) | External computer (`external/team_perception_external`) |
|---|---|---|
| Nodes | `qr_scanner`, `image_throttle`, `poster_detector` | `human_detector` (YOLO) |
| Needs | colour, depth, both `camera_info`s, TF, `/map` | only JPEG frames |
| Why there | Depth, TF and the map are big (~370 Mbit/s for raw colour and depth) and must stay local. QR decoding and the poster depth test are light. | YOLO is the heaviest step: 50–100 ms a frame on a CPU, and the robot has no usable GPU. |
| Heavy deps | none (no torch) | torch + ultralytics + `yolo11n.pt` |

The packages have different names (`team_perception` and `team_perception_external`), so both
can be installed on the same computer if needed (section 9).

## 2. Data flow

```
 ROBOT COMPUTER                                                  EXTERNAL COMPUTER
 ──────────────                                                  ─────────────────
 Orbbec camera (via lite3_domain_bridge_4)
   │ /camera/color/image_raw/compressed (JPEG, 30 Hz)
   ├──► qr_scanner ──► /qr/detections, /found "QR found"
   │
   └──► image_throttle (keeps 1 frame in 3)
          │ /camera/color/image_offboard/compressed   ───── Wi-Fi / LAN ─────►  human_detector
          │ 10 Hz, ~6 Mbit/s, BEST_EFFORT depth 1                               (YOLO, class "person")
          │                                                                         │
 /camera/depth/image_raw ─┐                                                         │
 both camera_info topics ─┤                                                         │
 TF (map→…→camera)       ─┼─► poster_detector  ◄── /humans/detections (JSON) ◄─────┘
 /map (transient local)  ─┘        │               a few KB/s, only frames with people
                                   └──► /posters/detections, /found "Human found"
```

Only two topics cross the link:

| Topic | Direction | Type | Size |
|---|---|---|---|
| `/camera/color/image_offboard/compressed` | robot → external | `sensor_msgs/CompressedImage` | ~6 Mbit/s at 10 Hz |
| `/humans/detections` | external → robot | `std_msgs/String` (JSON) | a few KB/s |

There is no custom protocol. Both computers join the same ROS 2 graph (same
`ROS_DOMAIN_ID`, Fast DDS discovery), and each node just subscribes to the topic by name.

## 3. Process breakdown: one frame, start to finish

1. **Camera → robot.** The Orbbec driver publishes a colour JPEG at 30 Hz, stamped with the
   capture time `t`. Depth is published separately, also stamped.
2. **QR (robot only).** `qr_scanner` decodes every frame locally. A new code is published on
   `/qr/detections` and `/found` gets `QR found`. QR never touches the external computer.
3. **Throttle (robot).** `image_throttle` forwards roughly every third JPEG **unchanged**,
   header stamp `t` included, to `/camera/color/image_offboard/compressed`. It publishes
   BEST_EFFORT with queue depth 1, so on a slow link old frames are dropped, not queued.
4. **Buffer depth (robot).** Meanwhile `poster_detector` keeps the last **2 s** of depth images,
   indexed by their stamps.
5. **YOLO (external).** `human_detector` decodes the newest frame, runs YOLO, and, if it finds at
   least one person, publishes one JSON message on `/humans/detections`:
   ```json
   {"stamp": t, "frame_id": "camera_color_optical_frame", "image_size": [640, 480],
    "count": 1, "people": [{"box": [x1, y1, x2, y2], "conf": 0.85,
                            "pixel": [u, v], "frame": "none", "position": null}]}
   ```
   `position` is `null` because depth isn't streamed out (`use_depth:=false`). Frames with no
   people produce no message.
6. **Pair by stamp (robot).** `poster_detector` looks up the depth image whose stamp is closest
   to `t`, and accepts it only within `max_dt_s` = 0.1 s. Because `t` is the **camera's**
   stamp, carried through the whole round trip, the computers' clocks don't need to be in
   sync, and the reply may arrive up to 2 s late (the buffer length) without losing the frame.
7. **Poster test (robot).** For each box: map it into the unaligned depth image (`DepthAlign`),
   run the edge-ring flatness test, place the torso in the `map` frame through TF, and (with
   `map_mode:=veto`) check the static map. The result is `poster`, `real` or `unknown`.
8. **Vote and report (robot).** Labels are tracked by position (0.5 m merge radius). When a
   track has **5 poster votes out of the last 8**, it is confirmed: one message on
   `/posters/detections`, one `Human found` on `/found`, and a CSV row in `~/poster_logs/`.

What this means in practice:

- No external computer, or a broken link: QR still works, but **no `Human found`** is ever
  published (no person boxes reach the robot).
- Round trip over 2 s: every detection is dropped with the warning
  `no depth image within max_dt_s of the detection`.
- Low frame rate: a poster seen for only a second or two may not collect 5 votes.

## 4. What the robot must provide

| Input | Type | Used by | If it's missing |
|---|---|---|---|
| Colour image: `/camera/color/image_raw/compressed` (or raw with `image_transport:=raw`) | `sensor_msgs/CompressedImage` or `Image` | all detection | Nothing is detected. `image_throttle` always needs the compressed topic. |
| A network link to the external computer, same `ROS_DOMAIN_ID`, about 6 Mbit/s | | `human_detector` | No `/humans/detections`, so **posters are never confirmed**. QR codes still work. |
| Depth image `/camera/depth/image_raw`, 16UC1 mm or 32FC1 m, plus its `/camera/depth/camera_info` (not needed if the depth is aligned to colour) | `sensor_msgs/Image`, `CameraInfo` | positions; the poster test | QR codes still reported, with no position. **Posters are never confirmed.** |
| TF `camera_color_optical_frame` → `camera_depth_optical_frame` (robot's `/tf_static`) | TF | colour-to-depth mapping | Mapping is up to 5 px off at 1 m. |
| Colour camera intrinsics: `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | positions | No positions. |
| TF from the camera optical frame to `map` (from localisation) | TF | positions in `map`; the map test | Positions come out in the camera frame (it says so in `frame`), and the map test is skipped. |
| Static map: `/map`, transient local, built **without people in it** | `nav_msgs/OccupancyGrid` | the poster map test (`map_mode`) | With the default `veto`, the depth test decides alone. |

Check every topic name with `ros2 topic list` and override it with a launch argument
(`ros2 launch team_perception robot.launch.py --show-args`, and the same for
`team_perception_external external.launch.py`).

**Unaligned depth: compensated in our code for now.** On the Lite3, `/camera/depth/image_raw`
is **not** aligned to the colour image: depth fx = 373 against colour fx = 462, and the two
sensors are 14 mm apart. The Orbbec driver runs on the shared lab bridge, which we can't log in
to, so alignment can't be turned on there yet. Until it can, every node maps colour pixels into
the depth image itself:
- It uses both `camera_info` topics and the TF between `camera_color_optical_frame` and
  `camera_depth_optical_frame` (on `/tf_static`). This is `qr_utils.DepthAlign`.
- Without that mapping, pixels near the left and right image edges are about 60 px off. In the
  2026-11-12 bag this gave about a quarter as many poster labels, and the poster was confirmed
  21 s late.
- Without the TF, the mapping still runs, but ignores the 14 mm offset: up to 5 px off at 1 m.
  The log warns.
- If depth-to-colour alignment is ever turned on in the driver, pass
  `depth_camera_info_topic:=''` (and the aligned `depth_topic`). To check whether depth is
  aligned: the depth topic's `camera_info` then matches `/camera/color/camera_info`.
- If depth or its `camera_info` is missing, the log says `depth not used: <reason>`.

## 5. Detector outputs

Every result is published on the **robot**, except `/humans/detections`, which comes from the
external computer. All result topics are `std_msgs/String` with the default QoS (reliable,
volatile, depth 10): **a listener that starts late misses earlier messages**, so start it before
the run.

| Detector | Runs on | Topic | When it publishes |
|---|---|---|---|
| `qr_scanner` | robot | `/found` | `QR found`, once per new code text |
| | | `/qr/detections` | JSON when a code is first read, then every 30 s (`repeat_after_s`) while still in view |
| `human_detector` | external | `/humans/detections` | JSON for every processed frame with a person in it (up to 10 a second). Real people **and** posters |
| `poster_detector` | robot | `/found` | `Human found`, once per new poster position (within 0.5 m = the same poster). Never for real people |
| | | `/posters/detections` | JSON when a poster is confirmed (5 of 8 votes), then every 60 s while still in view |
| `image_throttle` | robot | `/camera/color/image_offboard/compressed` | JPEG frames for the external computer (not a result) |

The mission logic should use `/found` or `/posters/detections`, not `/humans/detections`.

### `/found`

Plain text, no JSON. Other tools match these exact strings.

```
data: QR found
---
data: Human found
---
```

### `/qr/detections` (robot)

```json
{"text": "I", "pixel": [412.5, 133.0], "frame": "map",
 "position": [1.84, 0.62, 0.41], "snapshot": "/home/<user>/qr_logs/qr_1794488686123.jpg",
 "stamp": 1794488686.12}
```

| Field | Meaning |
|---|---|
| `text` | Decoded content of the code |
| `pixel` | Centre of the code in the colour image (px) |
| `frame` | `map` with TF; the camera optical frame without TF; `none` without depth |
| `position` | `[x, y, z]` in metres in `frame`, or `null` without depth |
| `snapshot` | Annotated image of the find, saved on the robot |
| `stamp` | Camera time of the image (s) |

### `/humans/detections` (external → robot)

The input to `poster_detector`, also useful for checking the link.

```json
{"stamp": 1794488687.09, "frame_id": "camera_color_optical_frame", "image_size": [640, 480],
 "count": 1, "people": [{"box": [7.0, 95.0, 46.0, 191.0], "conf": 0.719,
 "pixel": [26.5, 133.4], "frame": "none", "position": null}]}
```

| Field | Meaning |
|---|---|
| `stamp` | Camera stamp of the frame, copied unchanged: `poster_detector` pairs depth with it |
| `image_size` | Size of the image the boxes refer to (px) |
| `people[].box` | `[x1, y1, x2, y2]` in colour-image pixels |
| `people[].conf` | YOLO person confidence (0–1, minimum `conf` = 0.4) |
| `people[].pixel` | Pixel used for depth (torso) |
| `people[].frame`, `position` | `none` / `null` by default, because depth stays on the robot. Filled in only with `use_depth:=true` |

### `/posters/detections` (robot)

```json
{"stamp": 1794488687.09, "id": 1, "frame": "map", "position": [1.67, 0.77, -0.08],
 "votes": "5/5", "conf": 0.719, "box": [7.0, 95.0, 46.0, 191.0],
 "ring": "surface continues past the edges", "map": "on/next to a static surface in the map"}
```

| Field | Meaning |
|---|---|
| `id` | Track number. Can change if the poster is out of view for more than 30 s: use `position` to tell posters apart |
| `frame`, `position` | `map` with TF, otherwise the camera frame; metres |
| `votes` | Poster votes over this track's recent frames |
| `conf`, `box` | YOLO confidence and box of the frame that confirmed it |
| `ring` | Reason from the depth edge-ring test |
| `map` | Reason from the static-map test: `null` when it is off, `no tf to map` without TF |

### `/camera/color/image_offboard/compressed` (robot → external)

`sensor_msgs/CompressedImage`, published by `image_throttle`: the camera's JPEG frames,
unchanged (header stamp included), at most `offboard_rate_hz` (10) per second. BEST_EFFORT,
depth 1, so on a slow link old frames are dropped rather than queued. Only `human_detector`
should subscribe to it.

### Console logs and files on the robot

The launch terminal shows each find:

```
[qr_scanner]: QR found
[qr_scanner]: QR "I" at map (1.84, 0.62, 0.41)
[poster_detector]: Human found
[poster_detector]: POSTER #1 at map [1.67, 0.77, -0.08] (votes 5/5)
```

| Folder (robot) | Content | Created |
|---|---|---|
| `~/qr_logs/` | CSV row (`wall_time, text, frame, x, y, z, snapshot`) and an annotated JPEG per report | on the first code |
| `~/poster_logs/` | CSV row (`wall_time, poster_id, frame, x, y, z, votes, conf, ring, map`) per confirmed poster report. No images | on the first poster |

The external computer writes no files.

### Watching the outputs live

From the robot, or any computer on domain 4:

```bash
ros2 topic echo /found
ros2 topic echo /qr/detections
ros2 topic echo /posters/detections
ros2 topic echo /humans/detections      # raw person boxes, only while someone is in view
```

## 6. Integrating into the robot's software

1. **Start it** with the robot's other software, for example by including `robot.launch.py`
   from the robot's main launch file, with the topic names set as launch arguments. `qr:=false`
   or `posters:=false` turns off one half. Start `external.launch.py` on the external computer
   too, and check with `ros2 topic hz /humans/detections` on the robot that its results arrive
   while someone is in view.
2. **Before it starts**, make sure depth and both `camera_info` topics are published,
   localisation publishes TF to `map`, and the static map is on `/map`.
3. **Use the outputs.** For a counter or a score display, subscribe to `/found`. To mark finds
   on the map or to record them for scoring, subscribe to `/qr/detections` and
   `/posters/detections` and parse the JSON. For example:

   ```python
   import json
   from std_msgs.msg import String

   def on_poster(msg: String):
       p = json.loads(msg.data)
       if p['frame'] == 'map':
           mark_victim(p['position'][:2])      # x, y in the map

   node.create_subscription(String, '/posters/detections', on_poster, 10)
   ```

4. **CPU load and bandwidth.**
   - YOLO, the heaviest step, runs on the external computer: about 50–100 ms per 640 px frame
     on a laptop CPU, much less on a GPU (`device:=0`), limited to 10 frames a second
     (`rate_hz`).
   - The robot runs the QR decoder (up to 30 frames a second) and the depth-based poster test.
   - The link carries about 6 Mbit/s of JPEG out and a few KB/s of JSON back. On a slow link,
     lower `offboard_rate_hz`.
5. **Limits to know about.**
   - The poster thresholds have been checked on only two recorded bags so far (`teleop_run1`,
     `posters_0`): every confirmed poster was a real poster, with no false posters.
   - Beyond 4 m (`max_range_m`), depth is too noisy and a person counts as unknown, never as a
     poster.
   - Positions are only as good as localisation and the camera-to-base TF.

## 7. Network requirements

| Item | Value |
|---|---|
| ROS domain | `ROS_DOMAIN_ID=4` on both computers (the Lite3's domain) |
| ROS distro / DDS | ROS 2 Humble, Fast DDS (`rmw_fastrtps_cpp`, the default) on both |
| Subnet | Robot computer: `192.168.10.74` on the lab network (`eth1`). The external computer must be on the same lab network (`192.168.10.x`). See `robot_info.md` §1 |
| Bandwidth | ~6 Mbit/s up from the robot at 10 Hz. Wired, or 5 GHz Wi-Fi near the access point |
| Discovery | DDS multicast must pass between the two. If it is blocked, set unicast peers in a Fast DDS profile, or bridge only the two topics with `zenoh-bridge-ros2dds` |
| Clocks | No sync needed (pairing uses the camera stamp) |

Never use domain 4 for tests that shouldn't reach the robot: use another ID (e.g. 89) and
`ROS_LOCALHOST_ONLY=1`.

## 8. Setup

Copy only the inner package folders. **Don't copy `challenge_files/COLCON_IGNORE`**: it would
stop colcon from building the package.

### 8.1 Robot computer (onboard)

Ubuntu 22.04 with ROS 2 Humble already installed.

```bash
# 1. From the dev machine: copy the onboard package into the robot's workspace
scp -r challenge_files/onboard/team_perception <robot>:~/robot_ws/src/

# 2. On the robot: dependencies (apt asks for sudo; use --no-apt if they are already there)
bash ~/robot_ws/src/team_perception/install_deps.sh

# 3. Build from the workspace root, never from src/
cd ~/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select team_perception
source install/setup.bash

# 4. Join the robot's ROS graph and check the inputs exist
export ROS_DOMAIN_ID=4
ros2 topic list | grep -E "camera|/map|/tf"
```

`install_deps.sh` installs `cv_bridge`, `tf2`, OpenCV, numpy < 2, `pyzbar` + `libzbar0` (apt)
and `zxing-cpp` (pip), then checks the imports.

### 8.2 External computer

Ubuntu 22.04 with ROS 2 Humble already installed. An NVIDIA GPU helps but isn't required.

```bash
# 1. Copy the external package into a workspace on that computer
mkdir -p ~/ext_ws/src
cp -r challenge_files/external/team_perception_external ~/ext_ws/src/

# 2. Dependencies + YOLO weights (add --gpu on an NVIDIA machine for the CUDA torch build)
bash ~/ext_ws/src/team_perception_external/install_deps.sh

# 3. Build
cd ~/ext_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select team_perception_external
source install/setup.bash

# 4. Join the robot's ROS graph
export ROS_DOMAIN_ID=4
```

`install_deps.sh` installs torch/torchvision (CPU, or CUDA with `--gpu`), `ultralytics`,
numpy < 2, removes pip `setuptools` (it breaks colcon on Humble) and downloads
`yolo11n.pt` to `~/.cache/team_perception/`.

No internet on the external computer? Copy `~/.cache/team_perception/yolo11n.pt` (~5 MB) from
the dev machine to the same path, and install the pip packages from wheels downloaded elsewhere
(`pip3 download -d wheels ...`, then `pip3 install --user --no-index --find-links wheels ...`).

Tip: put `source /opt/ros/humble/setup.bash`, `source ~/<ws>/install/setup.bash` and
`export ROS_DOMAIN_ID=4` in `~/.bashrc` on both computers so every new terminal is ready.

## 9. Running

Start the robot side first, then the external side (either order works, but posters are only
detected once both are up).

```bash
# Robot computer
ros2 launch team_perception robot.launch.py
#   offboard_rate_hz:=5.0    slow Wi-Fi
#   posters:=false           QR scanner only (no external computer needed)

# External computer
ros2 launch team_perception_external external.launch.py
#   device:=0                first NVIDIA GPU
#   model:=yolo11s.pt        bigger model, if a GPU can afford it
```

Keep `rate_hz` on the external side at or below `offboard_rate_hz` on the robot: YOLO can't
process frames it never receives.

**No external computer available?** Install `team_perception_external` on the robot as well
(section 8.2 in the robot's workspace) and run YOLO there on the full stream, at the cost of
robot CPU:

```bash
ros2 launch team_perception robot.launch.py
ros2 launch team_perception_external external.launch.py \
    image_topic:=/camera/color/image_raw use_depth:=true rate_hz:=5.0
```

## 10. Checking the link

Run these in order. Each step depends on the one before.

| # | Where | Command | Expect |
|---|---|---|---|
| 1 | external | `ros2 topic list \| grep camera` | the robot's camera topics appear (discovery works) |
| 2 | external | `ros2 topic hz /camera/color/image_offboard/compressed` | ~10 Hz (= `offboard_rate_hz`) |
| 3 | robot | `ros2 topic hz /humans/detections` (stand in front of the camera) | a steady rate while someone is in view |
| 4 | robot | `ros2 topic echo /humans/detections --once` | JSON with `stamp`, `image_size` and `people` |
| 5 | robot | `ros2 topic echo /found` (show a person poster) | `Human found` after a few seconds |

## 11. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Step 1 fails: no robot topics on the external computer | Different `ROS_DOMAIN_ID`, different subnet, or multicast blocked | Check `echo $ROS_DOMAIN_ID` on both; same `192.168.10.x` subnet; otherwise unicast peers or `zenoh-bridge-ros2dds` |
| Topics listed but step 2 shows nothing | Robot side not running, or `posters:=false` | Start `robot.launch.py` with posters on |
| Step 2 well below 10 Hz | Weak Wi-Fi | Move closer / go wired, or lower `offboard_rate_hz` |
| Robot logs `no depth image within max_dt_s of the detection` | Round trip over 2 s, or depth not arriving | Check `ros2 topic hz /camera/depth/image_raw` on the robot; lower `offboard_rate_hz`; use a GPU or smaller `imgsz` on the external side |
| Robot logs `depth not used: <reason>` | A `camera_info` topic hasn't arrived yet | Check both `camera_info` topics exist |
| People detected but never `Human found` | Fewer than 5 poster votes in 8, no TF to `map` (tracks restart), or the poster is over 4 m away | Get closer, check TF `map → camera_color_optical_frame` |
| `ros2 topic echo` fails with `!rclpy.ok()` | Stale ROS daemon | `ros2 daemon stop; ros2 daemon start` |
| `colcon build` fails with `_distribution_fullname` | pip setuptools installed with torch | `pip3 uninstall -y setuptools` |

## 12. Debugging a run

The onboard build has no debug topics, viewers or `annotate`. To debug, record a rosbag on the
robot during the run, then replay it on a dev machine that has the full `team_perception` from
GitHub, which has the debug images and `/posters/classifications`.

Without `/map` and `/tf` in the bag, a replay **can't** reproduce the poster results: there are
no `map` positions, the static-map test is skipped, and a poster seen twice becomes two tracks.
Sections 12.1 and 12.2 make sure both get recorded.

### 12.1 Before recording: check that the map and TF exist

On the robot, with SLAM/localisation running (`slam_toolbox` for `map → odom`, Fast-LIO's
`odom_relay` for `odom → base_link`):

```bash
export ROS_DOMAIN_ID=4
ros2 topic info /map --verbose | grep -E "Publisher count|Durability"   # >= 1, TRANSIENT_LOCAL
ros2 run tf2_ros tf2_echo map camera_color_optical_frame               # prints a transform each second
ros2 run tf2_tools view_frames                                         # optional: frames_*.pdf of the tree
```

If `tf2_echo` says the `map` frame doesn't exist, SLAM/localisation isn't running. Start it
before recording, or the bag will have no `/tf` to `map`. The map must be built **without people
in it**, otherwise the static-map test treats them as part of the map.

### 12.2 Record on the robot

`/map` and `/tf_static` are published **once** (transient local), often long before recording
starts. A QoS override file makes the recorder ask for transient local, so it still gets them.
Create it once on the robot as `~/bags/qos_overrides.yaml`:

```yaml
/map:
  history: keep_last
  depth: 1
  reliability: reliable
  durability: transient_local
/tf_static:
  history: keep_last
  depth: 100          # one message per static publisher: keep them all
  reliability: reliable
  durability: transient_local
```

Then record, with the detectors and SLAM running:

```bash
ros2 bag record -o ~/bags/run_$(date +%m%d_%H%M) \
  --qos-profile-overrides-path ~/bags/qos_overrides.yaml \
  /camera/color/image_raw/compressed /camera/color/camera_info \
  /camera/depth/image_raw /camera/depth/camera_info \
  /tf /tf_static /map \
  /camera/color/image_offboard/compressed /humans/detections \
  /qr/detections /posters/detections /found /rosout
```

| Group | Topics | Needed for |
|---|---|---|
| Inputs | colour, depth, both `camera_info`s | re-running the detectors |
| Localisation | `/tf`, `/tf_static`, `/map` | positions in `map`, the static-map test, tracking across views |
| Link | `image_offboard`, `/humans/detections` | telling a link failure from a logic failure |
| Results | `/qr/detections`, `/posters/detections`, `/found`, `/rosout` | what the robot decided, plus its warnings |

- Raw depth makes the bag about **1 GB a minute**, so check the free space with `df -h ~` first.
  Recording also loads the robot's CPU: an earlier bag dropped to 15.9 fps.
- To check only *whether* the run worked, leave out the two `/camera/depth/*` topics and
  `/camera/color/image_raw/compressed`. Keep `/tf` and `/map`.
- **Can't record the map?** Save it separately while SLAM runs, and serve it on the dev machine
  in section 12.4:
  `ros2 run nav2_map_server map_saver_cli -f ~/bags/run_xxxx_map` (writes `.yaml` + `.pgm`).

### 12.3 Quick check on the robot

```bash
ros2 bag info ~/bags/run_xxxx
ls ~/qr_logs ~/poster_logs
```

Go down the table and stop at the first row that matches:

| `ros2 bag info` shows | Meaning |
|---|---|
| `/map` count 0 | The map wasn't captured: the static-map test can't be replayed. Use the saved map (12.2) |
| `/tf` count 0, or `tf2_echo` failed before the run | No `map` positions: tracking was in the camera frame |
| `/tf_static` count 0 | The colour↔depth offset is missing: mapping can be up to 5 px off |
| Colour or depth well below 30 Hz | Camera or recording overload: depth pairing fails more often |
| `image_offboard` count ≈ 1/3 of colour | Throttle OK |
| `/humans/detections` count 0 | Nothing came back from the external computer: link, domain or discovery (section 11) |
| `/humans/detections` > 0 but no `Human found` | The link works: the problem is the poster logic (12.5, mode A) |

### 12.4 Replay on the dev machine

```bash
scp -r <robot>:~/bags/run_xxxx <robot>:~/bags/qos_overrides.yaml \
    ~/robot_challenge_workspace/src/team_perception/test_runs/   # gitignored
cd ~/robot_challenge_workspace/src/team_perception/test_runs
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1   # never replay on domain 4: it would reach the robot
ros2 daemon stop; ros2 daemon start            # otherwise `topic echo` crashes
```

**Keeping `/map` and `/tf_static` on replay.** Start the nodes **first**, then play the bag with
the same override file. Without it, a node that starts after the bag's single `/map` message
never receives the map.

```bash
ros2 bag play run_xxxx --qos-profile-overrides-path qos_overrides.yaml --topics <list below>
```

Check that the replay is complete:

- `poster_detector` logs `Static map received: <w>x<h> @ <res> m`.
- `ros2 run tf2_ros tf2_echo map camera_color_optical_frame` prints transforms while the bag plays.
- Don't use `--clock`. The nodes pair by message stamps, and TF is looked up at those recorded
  stamps, so playing in wall time works.

If the bag has no `/map` but you saved one, serve it instead:

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=run_xxxx_map.yaml &
ros2 run nav2_util lifecycle_bringup map_server
```

### 12.5 Two replay modes: link problem or logic problem?

**A. Re-judge what the robot actually received.** Replay the recorded `/humans/detections` and
run only the dev `poster_detector`, with its debug output on:

```bash
ros2 run team_perception poster_detector --ros-args \
  -p publish_debug:=true -p image_transport:=compressed
ros2 bag play run_xxxx --qos-profile-overrides-path qos_overrides.yaml --topics \
  /humans/detections /camera/color/image_raw/compressed /camera/depth/image_raw \
  /camera/depth/camera_info /camera/color/camera_info /tf /tf_static /map
rqt_image_view /posters/debug_image
../team_perception/scripts/perception.sh echo classifications   # ring/map numbers per box
```

**B. Re-run YOLO as well.** Leave `/humans/detections` and every result topic out of `--topics`.
Run the dev `human_detector` and `qr_scanner` (both with `-p image_transport:=compressed`) and
`poster_detector` as above. Add `-r 0.5` to `ros2 bag play`, because YOLO takes about 100 ms a
frame on the dev CPU.

| A | B | Conclusion |
|---|---|---|
| no poster | poster found | The boxes didn't reach the robot in time: lower `offboard_rate_hz`, improve the link, or use a GPU on the external computer |
| no poster | no poster | Logic or data problem. `/posters/classifications` shows which: the ring test said `real`/`unknown`, the map test vetoed it, it was more than 4 m away, or the track got fewer than 5 of 8 votes |
| poster found | — | Compare with the bag's recorded `/posters/detections`. If the robot missed it live, check `/rosout` for `no depth image within max_dt_s` (CPU load or depth gaps) |

For QR codes, compare the `/qr/detections` from mode B with the ones recorded in the bag.

### 12.6 Clean up after a replay

```bash
pgrep -fa "team_perception/lib|bag play|topic echo|map_server"   # kill these PIDs (kill <pid>)
```

`ros2 run` is only a wrapper: kill the child process under `install/.../lib/team_perception/`
too. Keep bags that show a bug as regression data, and note their known results in the repo's
`AGENTS.md`.
