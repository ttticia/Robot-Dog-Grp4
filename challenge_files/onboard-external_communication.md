# Integrating `team_perception` on the robot

What the perception package needs from the robot, what it publishes, and how to use its
outputs. For installing and building, see [../README.md](../README.md). A one-page
checklist of the robot data it needs is in [data_required.md](data_required.md).

## In short

```bash
ros2 launch team_perception robot.launch.py          # defaults match the Lite3
```

One launch file starts three nodes. To know when the robot has found something, listen
to **`/found`**: it carries `QR found` or `Human found`, once per new find. For *what*
was found and *where*, read `/qr/detections` and `/posters/detections` (JSON in a
`std_msgs/String`).

```
 camera colour ──┬─────────────► qr_scanner ───────► /qr/detections, /found "QR found"
                 │
                 └─► human_detector ─► /humans/detections ─► poster_detector ─► /posters/detections
                                                                   ▲            /found "Human found"
 camera depth + depth camera_info ─► qr_scanner, human_detector, poster_detector
 camera_info, TF camera→map, /map ───────────────────────────────┘
```

## What the robot must provide

| Input | Type | Used by | If it's missing |
|---|---|---|---|
| Colour image: `/camera/color/image_raw/compressed` (or raw with `image_transport:=raw`) | `sensor_msgs/CompressedImage` or `Image` | all detection | Nothing is detected. |
| Depth image `/camera/depth/image_raw`, 16UC1 mm or 32FC1 m, plus its `/camera/depth/camera_info` (not needed if the depth is aligned to colour) | `sensor_msgs/Image`, `CameraInfo` | positions; the poster test | QR codes still reported, with no position. **Posters are never confirmed.** |
| TF `camera_color_optical_frame` → `camera_depth_optical_frame` (robot's `/tf_static`) | TF | colour-to-depth mapping | Mapping is up to 5 px off at 1 m. |
| Colour camera intrinsics: `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | positions | No positions. |
| TF from the camera optical frame to `map` (from localisation) | TF | positions in `map`; the map test | Positions come out in the camera frame (it says so in `frame`), and the map test is skipped. |
| Static map: `/map`, transient local, built **without people in it** | `nav_msgs/OccupancyGrid` | the poster map test (`map_mode`) | With the default `veto`, the depth test decides alone. |

**Unaligned depth: compensated in our code for now.** On the Lite3, `/camera/depth/image_raw`
is **not** aligned to the colour image: depth fx = 373 against colour fx = 462, and the two sensors are 14 mm apart.
The Orbbec driver runs on the shared lab bridge, which we can't log in to, so alignment can't be turned on there
yet. Until it can, every node maps colour pixels into the depth image itself:
- It uses both `camera_info` topics and the TF between `camera_color_optical_frame` and
  `camera_depth_optical_frame` (on `/tf_static`). This is `qr_utils.DepthAlign`.
- Without that mapping, pixels near the left and right image edges are about 60 px off. In the 2026-11-12 bag this
  gave about a quarter as many poster labels, and the poster was confirmed 21 s late.
- Without the TF, the mapping still runs, but ignores the 14 mm offset: up to 5 px off at 1 m. The log warns.
- If depth-to-colour alignment is ever turned on in the driver, pass `depth_camera_info_topic:=''` (and the aligned
  `depth_topic`). To check whether depth is aligned: the depth topic's `camera_info` then matches
  `/camera/color/camera_info`.
- If depth or its `camera_info` is missing, the log says `depth not used: <reason>`.

Check every topic name with `ros2 topic list` and override with launch arguments
(`ros2 launch team_perception robot.launch.py --show-args`).

## What it publishes

All outputs use the default QoS (reliable, volatile, depth 10). **A node that subscribes late misses
earlier messages**, so start listeners before the run or together with the launch.

### `/found` — `std_msgs/String`

The simplest interface: plain text, no JSON.

| Message | Sent by | When |
|---|---|---|
| `QR found` | `qr_scanner` | Once per new code text, per run. |
| `Human found` | `poster_detector` | Once per new **poster** position (posters within `merge_radius_m` = 0.5 m count as the same one). Real people never trigger it. |

Other tools match these exact strings, so they must not change.

### `/qr/detections` — `std_msgs/String` (JSON)

One message when a code is first read, then again every `repeat_after_s` (30 s) while it is still in view.

```json
{"text": "I", "pixel": [412.5, 133.0], "frame": "map",
 "position": [1.84, 0.62, 0.41], "snapshot": "/home/<user>/qr_logs/qr_1794488686123.jpg",
 "stamp": 1794488686.12}
```

| Field | Meaning |
|---|---|
| `text` | Decoded content of the code. |
| `pixel` | Centre of the code in the colour image (px). |
| `frame` | Frame of `position`: `map` normally, or the camera optical frame when there is no TF. Without depth it is `none`. |
| `position` | `[x, y, z]` in metres, or `null` without depth. |
| `snapshot` | Annotated image of the find, saved on the robot. |
| `stamp` | Time of the camera image (s). |

### `/posters/detections` — `std_msgs/String` (JSON)

One message when a poster of a person is **confirmed** (by default 5 of its last 8 frames
say poster), then again every `repeat_after_s` (60 s) while it is still in view.

```json
{"stamp": 1794488687.09, "id": 1, "frame": "map", "position": [1.67, 0.77, -0.08],
 "votes": "5/5", "conf": 0.719, "box": [7.0, 95.0, 46.0, 191.0],
 "ring": "surface continues past the edges", "map": "on/next to a static surface in the map"}
```

| Field | Meaning |
|---|---|
| `id` | Track number. It can change if the poster is out of view for more than 30 s, so use `position` to tell posters apart. |
| `frame`, `position` | As above: `map` if TF is available, otherwise the camera frame. |
| `votes` | Poster votes over the recent frames of this track. |
| `conf` | YOLO person confidence (0–1). |
| `box` | `[x1, y1, x2, y2]` in colour-image pixels. |
| `ring`, `map` | Short reasons from the depth-edge test and the static-map test. `map` is `null` when the map test is off, and `no tf to map` when there is no TF. |

### `/humans/detections` — `std_msgs/String` (JSON)

The raw person detections that feed `poster_detector`: up to 10 messages a second, and only in frames with
someone in view. It includes **real people as well as posters**, so the mission logic should
normally use `/posters/detections` instead.

```json
{"stamp": 1794488687.09, "frame_id": "camera_color_optical_frame", "image_size": [640, 480],
 "count": 1, "people": [{"box": [7.0, 95.0, 46.0, 191.0], "conf": 0.719,
 "pixel": [26.5, 133.4], "frame": "map", "position": [1.67, 0.77, -0.08]}]}
```

## Files written on the robot

These are created on the first find, outside the workspace:

| Folder | Content |
|---|---|
| `~/qr_logs/` | `qr_<timestamp>.csv` (one row per report) and an annotated snapshot per report. |
| `~/poster_logs/` | `posters_<timestamp>.csv` (one row per confirmed poster report). No images. |

## Integrating it

1. **Start it** with the robot's other software, for example by including
   `robot.launch.py` from the robot's main launch file, with the topic names set as launch
   arguments. `qr:=false` or `posters:=false` turns off one half.
2. **Before it starts**, make sure depth and both `camera_info` topics are published, localisation
   publishes TF to `map`, and the static map is on `/map`.
3. **Use the outputs.** For a counter or a score display, subscribe to `/found`. To mark finds on the
   map or to record them for scoring, subscribe to `/qr/detections` and `/posters/detections` and
   parse the JSON. For example:

   ```python
   import json
   from std_msgs.msg import String

   def on_poster(msg: String):
       p = json.loads(msg.data)
       if p['frame'] == 'map':
           mark_victim(p['position'][:2])      # x, y in the map

   node.create_subscription(String, '/posters/detections', on_poster, 10)
   ```

4. **CPU load.** Everything runs on the CPU. YOLO takes about 50–100 ms per 640 px frame on the dev laptop, and
   `human_detector` is limited to 10 frames a second (`rate_hz`). Lower `rate_hz` or `imgsz` if the robot's
   computer can't keep up. The QR decoder runs at up to 30 frames a second.
5. **Limits to know about.**
   - The poster thresholds have been checked on only one teleop bag so far: one poster, confirmed
     correctly, no false posters.
   - Beyond 4 m (`max_range_m`), depth is too noisy and a person counts as unknown, never as a poster.
   - Positions are only as good as localisation and the camera-to-base TF.
