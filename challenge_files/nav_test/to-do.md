# To-do: tests to run on the robot

Changes and checks that can only be done or confirmed on the real Lite3. Each item says why
it's needed, what it depends on, the steps, and how to tell whether it passed. Most of the
code for section B is **not written yet**: the item says so.

Background: [../robot_info.md](../robot_info.md) (robot facts),
[../onboard-external_communication.md](../onboard-external_communication.md) (perception
setup, debugging with bags), [maze_explorer/README.md](maze_explorer/README.md) (navigation).

## How to use this file

- When you run an item, change its status in the overview table and add a line to the
  **Test log** at the end: date, robot number, result, bag name.
- Status values: `TODO`, `IN PROGRESS`, `PASSED`, `FAILED`, `BLOCKED` (say what blocks it).
- Record a bag for every test that involves motion (see A0); a failed test without a bag
  usually has to be run again.
- When an item passes and changes how things are set up, update the matching doc
  (`robot_info.md`, `onboard-external_communication.md`, the config comments) and the
  repo's `AGENTS.md`.

### Safety, before every test with motion

From `robot_info.md` §6:
- Someone holds the handheld controller, with its STOP button, next to the robot for the whole test.
- Only one `/cmd_vel` publisher at a time: check `ros2 topic info -v /cmd_vel`.
- Start at low speed (≤ 0.25 m/s) in an open area, away from people and the maze walls.
- Gait changes (especially H-Step) only with the lab operator's agreement.
- Use `ROS_DOMAIN_ID=4` only on the robot's network; for replays use another domain and
  `ROS_LOCALHOST_ONLY=1`.

## Overview

| ID | Item | Status | Depends on |
|---|---|---|---|
| A0 | Record a reference bag (flat floor + terrain) | TODO | — |
| A1 | Align depth to colour in the camera driver | TODO | lab operator, system-computer login |
| A2 | Switch our perception nodes to the aligned depth | TODO | A1 |
| B0 | Measure tilt: does `odom → base_link` carry roll/pitch? | TODO | A0 |
| B1 | Leveled scan frame `base_stabilized` + mount-bias correction | TODO (code not written) | B0 |
| B2 | Tilt gate for SLAM scans | TODO (code not written) | B0 |
| B3 | Ground removal | TODO (code not written) | B1 |
| B4 | 3D voxel obstacle layer | TODO (config not written) | B3 |
| B5 | Depth camera as a second obstacle source | TODO (code not written) | B3, ideally A1 |
| B6 | Gait, speed and recovery settings | TODO (decision needed) | operator |
| C1 | End-to-end perception link (robot ↔ external) | TODO | — |
| C2 | Poster thresholds on robot data with map + TF | TODO | C1, ideally A2 |
| C3 | Camera pose accuracy while walking | TODO | B0 |
| C4 | Recording load: does a bag delay the lidar? | TODO | — |

---

## A. Camera and data

### A0. Record a reference bag

**Why.** Almost every item below is checked by replaying a bag. One bag on flat floor and one
on the terrain give before/after data for B1–B5 and C2–C3.

**Steps.**
1. Start SLAM/localisation (`lite3_launcher` or `maze_explore.launch.py`) and check
   `ros2 run tf2_ros tf2_echo map base_link` prints transforms.
2. Start the robot from **level ground**, standing still for a few seconds (Fast-LIO's world
   frame is its start pose; see B1).
3. Record (the QoS override for `/map` and `/tf_static` is in
   `onboard-external_communication.md` §12.2):
   ```bash
   ros2 bag record -o ~/bags/terrain_$(date +%m%d_%H%M) \
     --qos-profile-overrides-path ~/bags/qos_overrides.yaml \
     /livox/points /livox/imu /tf /tf_static /map /scan /odom /joint_states \
     /camera/color/image_raw/compressed /camera/color/camera_info \
     /camera/depth/image_raw /camera/depth/camera_info /rosout
   ```
4. Operator walks the robot: 20 s standing still on flat floor, 30 s walking on flat floor,
   then over the terrain (slopes, blocks), including turns in place.

**Pass.** `ros2 bag info` shows every topic with a plausible count, `/map` ≥ 1 and
`/tf_static` ≥ 1. If `/livox/points` arrives late (see C4), record a second bag without depth.

### A1. Align depth to colour in the camera driver

**Why.** The Orbbec Gemini 335 publishes depth **not aligned** to colour (depth fx 373 against
colour fx 462, sensors 14 mm apart; `robot_info.md` §3.1). Our nodes work around it with
`qr_utils.DepthAlign`, a temporary fix that ignores the small rotation between the sensors
and lens distortion, and costs CPU. Alignment in the driver removes the workaround, and gives
every other user of depth (Nav2, B5) pixel-matched depth. The lab operator has allowed it
(`robot_info.md` §10, question 4).

**Needs.**
- Login to the robot's **system computer**, where the camera driver runs:
  `192.168.10.<60 + robot number>` (robot number, not group number), user `user`, password
  from the lab operator. Unconfirmed; `.62` looks like robot 2's.
- The driver's autostart: `~/.config/tmuxinator/lite3.yml` on that computer, which runs
  `orbbec_camera gemini_330_series.launch.py`.
- Agreement with the lab operator on **when**: restarting the driver stops the camera for
  everyone using this robot. Note the original launch line so it can be restored.

**Steps.**
1. On the system computer, find the exact parameter names in the installed driver version
   (don't trust names from memory or the internet):
   ```bash
   ros2 launch orbbec_camera gemini_330_series.launch.py --show-args \
     | grep -i -B1 -A3 "registration\|align"
   ```
   In Orbbec's ROS 2 SDK it is usually `depth_registration` (true/false) and `align_mode`
   (`HW` or `SW`). Prefer `HW`: it runs on the camera, while `SW` costs CPU on the system
   computer.
2. Back up and edit `~/.config/tmuxinator/lite3.yml`: add `depth_registration:=true` (and
   `align_mode:=HW` if it exists) to the camera launch line. Keep colour and depth at
   640×480 30 Hz; if hardware alignment rejects that combination, the driver log says so.
3. Restart the camera driver (its tmux window) with the operator, and watch its log for errors.

**Check, from our robot computer (domain 4).**
```bash
ros2 topic list | grep -i depth                       # did a new topic appear, or the same one?
ros2 topic echo /camera/depth/camera_info --once --field k
ros2 topic echo /camera/color/camera_info --once --field k
ros2 topic echo /camera/depth/camera_info --once --field header.frame_id
ros2 topic hz /camera/depth/image_raw                 # still ~30 Hz?
```

**Pass.**
- The aligned depth's `camera_info` has the **same** `k` (fx ≈ 462), width and height as
  colour, and its `frame_id` is the colour optical frame. Write down the topic name: it may be
  `/camera/depth/image_raw` again, or a new one.
- Depth still at ~30 Hz, with no new driver errors.
- Visual check: in `rqt_image_view`, edges of a door frame or a box are at the same pixels in
  colour and depth, including near the left and right image edges (unaligned, they were
  ~60 px off there).
- The system computer's CPU (`top`) hasn't jumped (it would with `SW` alignment).

**Rollback.** Restore the backed-up `lite3.yml` and restart the driver.

### A2. Switch our perception nodes to the aligned depth

**Depends on** A1 passing.

**Steps.**
1. Launch with the aligned depth and no depth `camera_info` (= depth already aligned):
   ```bash
   ros2 launch team_perception robot.launch.py \
     depth_topic:=<aligned depth topic from A1> depth_camera_info_topic:=''
   ```
2. Run C2's poster test and a QR code at a known position, and compare with the same test using
   the old `DepthAlign` settings (or a bag from before A1).

**Pass.** Same or better poster labels; QR and poster positions within a few cm of the
`DepthAlign` results; no `depth not used` warnings.

**Afterwards** (only after a few successful runs): make `depth_camera_info_topic:=''` and the
aligned topic the defaults in both launch files, then remove `DepthAlign`/`DepthAlignState`
as described in `AGENTS.md` ("Unaligned depth (TEMPORARY fix)"), keeping every copy of
`qr_utils.py` identical. Update `robot_info.md` §3.1 and `data_required.md`.

---

## B. Navigation on uneven terrain

**The problem.** `pointcloud_to_laserscan` keeps points between −0.15 and +0.08 m in
`base_link` ([maze_explorer/config/scan.yaml](maze_explorer/config/scan.yaml)). That band tilts
with the body. The floor sits 0.20 m below the band and 0.15 m terrain blocks only 0.05 m below
it, so nose-down pitch turns them into phantom walls:

| Body pitch | Flat floor becomes a wall at | 0.15 m block becomes a wall at |
|---|---|---|
| 2° | 5.7 m | 1.4 m |
| 5° | 2.3 m | 0.6 m |
| 10° | 1.1 m | 0.3 m |

Nose-up pitch does the opposite: the 0.45 m walls drop out of `/scan` beyond ~2.9 m at 5°.
Tilted scans also smear `slam_toolbox`'s 2D map (which `poster_detector` uses too), and the
lidar can't see the floor within ~1.25 m ahead, so low obstacles vanish as the robot reaches
them. Fast-LIO odometry itself copes with tilt.

### B0. Measure the tilt

**Why.** B1–B3 need `odom → base_link` to carry real roll and pitch, and their thresholds
should come from measured values, not guesses.

**Steps.**
1. While the operator walks the robot: `ros2 run tf2_ros tf2_echo odom base_link` and watch
   the RPY line.
2. From the A0 bag: plot roll and pitch of `odom → base_link` over time (standing, flat walk,
   terrain), and note the floor height in `base_link` while standing (expected ≈ −0.35 m,
   with the floor appearing to rise ~3 cm/m, a ~1.7° bias).
3. Replay the A0 bag and view `/scan` over `/map` in RViz: note where phantom arcs appear.

**Pass / outcome.** Write down: does RPY move (yes/no); typical and maximum |roll|, |pitch|
on flat floor and on terrain; the standing bias. If RPY stays at 0, `odom_relay` is planar:
B1/B2 must take orientation from `/fastlio/odometry` plus the static lidar mount instead.

### B1. Leveled scan frame `base_stabilized`

**How it works.** A new TF frame with `base_link`'s x, y, z and yaw, but roll = pitch = 0, as a
second child of `odom`. `pointcloud_to_laserscan` judges point heights in that frame, so the
band stays horizontal while the body rocks. The 3D points don't change.

```
odom ─┬─► base_link          (Fast-LIO, 6-DoF: tilts with the body)
      └─► base_stabilized    (new: roll = pitch = 0)
```

**Needs.**
- B0: TF with roll/pitch.
- **Robot started on level ground.** Fast-LIO's world frame is its start pose
  (`odom → lidar_init` = the lidar mount), so `odom` is only as level as the start.
- The ~1.7° standing bias removed: fit a plane (RANSAC) to floor points in `base_link` with
  the robot standing on flat floor, and correct the pitch/roll of the static
  `base_link → livox_frame` TF, or add the offset in the node.
- **Code (not written):** a small rclpy node: on each `/odom`, publish TF
  `odom → base_stabilized` with the same position, yaw only, and **the odom message's stamp**
  (a wrong stamp re-introduces the tilt). ROS-free logic in a `*_utils.py` with unit tests.
- **Config:** `scan.yaml` → `target_frame: base_stabilized`. Band values, costmap
  `robot_base_frame` and `slam_toolbox` `base_frame` can stay as they are.

**Test.**
1. Replay A0's terrain bag through `pointcloud_to_laserscan` with `target_frame: base_link` and
   then `base_stabilized`; compare in RViz (fixed frame `odom`).
2. Live, at low speed, on the terrain: watch `/scan` and the local costmap.

**Pass.** No phantom arcs of floor ahead of the robot at the pitches measured in B0; walls stay
steady while the robot pitches; standing still on flat floor, the floor is level in
`base_stabilized` (no ~3 cm/m rise).

**Limits.** Removes the robot's tilt, not the ground's: a rising ramp still enters the band
(B3 fixes that). Fast-LIO's z drift still moves the band slowly.

### B2. Tilt gate for SLAM scans

**How it works.** A relay node looks up `odom → base_link` at each scan's stamp and republishes
it on `/scan_slam` only if |roll| and |pitch| are below a limit and the pitch rate is low.
`slam_toolbox` reads `/scan_slam`; the costmaps keep reading every scan. SLAM only adds a scan
every 0.2 m / 0.2 rad anyway, so skipped scans are cheap. It keeps bad scans out of `/map`,
which is permanent and is also used by `poster_detector`.

**Needs.**
- B0 (TF with roll/pitch). Limits from B0's numbers (start around 4° and a few °/s).
- Tilt from TF or `/livox/imu`, **never `/imu/data`**: its steady-clock stamps are
  ~1.79 × 10⁹ s off the lidar's (`robot_info.md` §3.4).
- **Code (not written):** relay node; `slam_toolbox.yaml` → `scan_topic: /scan_slam`.

**Pass.** Log the share of dropped scans. Under ~30%, and a map from a terrain run without the
smeared or doubled walls of the same run without the gate. Over ~30%: the limit is too tight
(SLAM will lose track in turns).

### B3. Ground removal

**How it works.** Instead of a fixed band, an obstacle is a point more than `step_max` above
the **local ground**. In `base_stabilized`, put points in a 2D grid (~0.2 m cells), take a low
percentile of z per cell as the ground, smooth it over neighbouring cells, and keep points more
than `step_max` (start at 0.20 m: the 0.15 m blocks are walkable) above it. Publish them as a
`PointCloud2` for `pointcloud_to_laserscan` (wide band) or for B4.

**Needs.**
- B1.
- `/livox/points` (`PointCloud2`, xyz at byte offsets 0/4/8, `point_step` 22: reads with a
  numpy structured dtype).
- **Code (not written):** vectorised numpy (~20,000 points at 10 Hz), ROS-free logic with
  unit tests. Alternative: Patchwork++ or `linefit_ground_segmentation` (ROS 2 wrappers, built
  from source; no apt packages).
- Cells with no floor returns (the 1.25 m blind zone ahead, everything behind) have no ground
  estimate: fall back to the B1 band there.

**Pass.** On a ramp or slope in the terrain bag: no phantom wall at the ramp; blocks taller
than `step_max` and the maze walls still marked. Processing time per scan well under 100 ms.

### B4. 3D voxel obstacle layer

**How it works.** The 2D obstacle layer clears every cell a scan ray passes. When a low block
falls into the lidar's blind zone, rays to the wall behind it pass *over* it and clear its
cell just as the robot arrives. A voxel layer clears only the 3D cells a ray goes through, so
the block stays marked.

**Needs.**
- B3's obstacle cloud. The voxel layer's heights are absolute in `odom`, which is wrong on
  ramps or with z drift unless the cloud is already ground-removed.
- `nav2_costmap_2d::VoxelLayer` (already installed). Optional: the Spatio-Temporal Voxel
  Layer (`ros-humble-spatio-temporal-voxel-layer`, apt, needs sudo), whose voxels expire after
  a set time instead of staying stuck.
- **Config (not written):** in the local costmap of
  [maze_explorer/config/nav2_params.yaml](maze_explorer/config/nav2_params.yaml), replace
  `obstacle_layer` with a `VoxelLayer` (`z_resolution` 0.05, `z_voxels` 16, `PointCloud2`
  source on the obstacle cloud, wide min/max obstacle height). Keep `/scan` for SLAM.

**Pass.** Walking slowly toward a low box (~0.25 m): it stays in the local costmap until the
robot has passed it. Check latency: `collision_monitor` ignores data older than 0.5 s.

### B5. Depth camera as a second obstacle source

**How it works.** The camera is 0.42 m up and pitched 20° down, so it sees the floor much
closer than the lidar. Turn depth images into a cloud, downsample, remove the ground (B3), and
add it as a second observation source in the local costmap.

**Needs.**
- `ros-humble-depth-image-proc` (apt, sudo) or a numpy deprojection node; the depth topic and
  its `camera_info` (works with or without A1: the cloud uses the depth intrinsics).
- Throttle to 5–10 Hz and downsample (e.g. to 160×120): full-rate depth is ~9 million points/s.
- CPU headroom on the robot, which already runs `qr_scanner` and `poster_detector` on depth.

**Pass.** In RViz (fixed frame `odom`), the depth cloud of a low box lines up with the lidar's
points of the same box; the box is in the costmap at < 1.25 m where the lidar no longer sees
it. Measure how close to the robot the camera actually sees the floor (depends on the
Gemini 335's real field of view). Keep `obstacle_max_range` ≈ 2.5 m (depth is noisy beyond 3 m).

### B6. Gait, speed and recovery

**Decision needed.** [maze_explorer/config/supervisor.yaml](maze_explorer/config/supervisor.yaml)
and the launch file default to `gait: h_step`, but H-Step made the robot unsteady on 2026-11-12
(`robot_info.md` §10, question 3) and is only allowed with the operator's agreement. Either
default to `rug_general` (the rough-ground gait) and use H-Step only where needed, or confirm
with the operator that H-Step is safe now.

**Steps.**
1. Agree the gait with the operator; confirm the running gait with
   `ros2 run maze_explorer step_height` (foot lift) and set `gait_min_lift_m`.
2. Keep `max_velocity` at 0.25 m/s on terrain; check the velocity smoother's acceleration
   limits (hard accelerations pitch the body).
3. Check that the behaviour tree's clear-costmap recovery runs, so a phantom wall can't close
   a doorway for good: put a box in a doorway, let the costmap mark it, remove it, and see the
   robot recover.

**Pass.** Terrain crossed at the chosen gait without falls or fall-protection stops; a blocked
doorway is recovered from.

---

## C. Perception on the robot

### C1. End-to-end perception link

Follow `onboard-external_communication.md` §10 (link checks) on the real network: offboard
stream ~10 Hz on the external computer, `/humans/detections` arriving on the robot, and
`Human found` on `/found` in front of a poster. **Pass:** all five checks, and no
`no depth image within max_dt_s` warnings during a 2-minute run.

### C2. Poster thresholds on robot data

The poster thresholds are only checked on two bags (`teleop_run1`, `posters_0`), and neither
had `/map`. Record a run with posters **and** real people, with `/map` and `/tf`
(`onboard-external_communication.md` §12.2), replay it with `/posters/classifications`, and
check labels by hand. **Pass:** no real person confirmed as a poster; every poster within 4 m
confirmed; each physical poster gives one `Human found` (no duplicates from the map-height
spread seen in `posters_0`).

### C3. Camera pose accuracy while walking

In `posters_0`, the torso height of the same poster varied by ~0.5 m in `map`. Check whether
body pitch, TF timing or Fast-LIO z drift causes it: put a QR code at a measured position,
walk past it on flat floor and on terrain, and compare the reported positions.
**Pass:** positions within ~0.2 m of the measured one. If not, check the TF stamp used against
the image stamp, and whether B0 shows large pitch at those moments.

### C4. Recording load

While raw depth was being recorded, `/livox/points` arrived ~1.2 s late, so Nav2's
`collision_monitor` (0.5 s timeout) ignored it (`robot_info.md` §3.2). Measure the lidar
delay with and without depth recording (`ros2 topic delay /livox/points`). **Pass / outcome:**
know which topics can be recorded during an autonomous run without starving navigation; if
depth can't, record it only in teleop tests.

---

## Test log

| Date | Robot | Item | Result | Bag / notes |
|---|---|---|---|---|
| | | | | |
