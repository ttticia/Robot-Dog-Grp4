# Robot interface reference: DEEP Robotics Lite3

This is the reference for writing code that talks to our Lite3 when you can't log in to the
robot. It lists every ROS topic the robot provides, the format of each message, how motion
commands reach the robot (down to the UDP packets), and what software you need to build
against it.

- **Sources:** live inspection of the robot on 2026-11-13 (`ros2 topic`, `ros2 node info`,
  sampled messages), the manufacturer guides (local copies in
  `src/navigation/lite3-maze/docs/lite3_development/`: Motion Host Communication Interface
  V1.1.0, Motion Development Guide V2.2.1, Perception Development Guide V2.2.3, SDK Mode V1.0.0,
  Kit Installation V0.0.1), the manufacturer bridge
  source (`DeepRoboticsLab/Lite3_ROS`, branch `ros2-foxy`, commit `e4ab5dc`), the lab launcher
  `~/ros2_ws/src/lite3_launcher`, and our hardware trials of 2026-11-12.
- **How sure each fact is:** **[live]** means seen on the robot. **[doc]** means from the
  manufacturer documents or source. **[unverified]** means believed but not yet tested on
  our robot.
- **Related:** `agent.md` (workspace and build guide), `src/challenge_files/onboard-external_communication.md`
  (perception outputs), `src/navigation/lite3-maze/README.md` (maze autonomy).

---

## 1. System overview

```
 your laptop / robot computer (ROS 2 Humble, ROS_DOMAIN_ID=4)
        │  /cmd_vel (Twist), /simple_cmd (MotionSimpleCMD)          ▲ sensor topics, /tf_static
        ▼                                                            │
 ┌───────────────────────────────── lite3_domain_bridge_4 ──────────┴──────┐  shared lab node,
 │ forwards commands to UDP · republishes drivers + motion-host state     │  always running,
 └───────┬───────────────────────────────────────────────▲─────────────────┘  DON'T modify
         │ UDP → 192.168.1.120:43893                     │ UDP ← motion host reports
         ▼                                               │
   Motion host (manufacturer locomotion firmware) ───────┘
   Sensors: Livox Mid-360 lidar, Orbbec RGB-D camera, body IMU, joint encoders, ultrasound
```

| Item | Value |
|---|---|
| Robot | DEEP Robotics Jueying Lite3 quadruped. Body 548 × 370 mm; upper/lower leg 200/210 mm; foot radius 20 mm [doc] |
| Standing height | `base_link` is about 0.35 m above the ground (0.34751 m in the manufacturer source) [doc] |
| Robot computer (runs our code) | Ubuntu 22.04, ROS 2 **Humble**. `192.168.10.74` on the lab network (`eth1`); `192.168.1.74` on the robot's internal network (`eth0`) [live] |
| Motion host | `192.168.1.120`. Receives commands on **UDP 43893** [doc][live] |
| Manufacturer perception host | `192.168.1.103` (Jetson Xavier NX in the Pro/LiDAR edition; UDP 43899 for app/AI toggles). By default the motion host reports its state **only** to this host, port 43897 (set in `~/jy_exe/conf/network.toml` on the motion host). We don't use the host directly [doc] |
| Manufacturer navigation host | `192.168.1.106` (mentioned in the SDK guide) [doc, not seen on our network] |
| Wide-angle camera stream | `rtsp://192.168.1.120:8554/test`. Only the first client that connects receives it [doc, unverified on our robot] |
| Camera model | The manufacturer guides describe an Intel RealSense D435i. **Our robot has an Orbbec Gemini 335** (driver launch file `orbbec_camera gemini_330_series.launch.py`) [lab operator, 2026-11-13], so the guides' RealSense instructions (`realsense-viewer`, `align_depth.enable`) don't apply |
| `lite3_domain_bridge_4` host | **`192.168.1.50`** (robot network) / **`192.168.10.62`** (lab network). A separate guest (VM/container) on the same Proxmox host as our robot computer (ping ≈ 0.03 ms), not a process on our computer. Found by matching its DDS participant (prefix `01.0f.5c.28.0b.02.23.38.01.00.00.00`) to its locators, 2026-11-13 [live]. No hostname. Per the lab operator (2026-11-13), the camera driver runs on the robot's *system computer* at `192.168.10.<60 + robot number>` (robot number, **not** group number), user `user` (password from the lab operator); its autostart is `~/.config/tmuxinator/lite3.yml`. `.62` suggests this is that machine for robot 2. Unconfirmed |
| ROS domain | **`ROS_DOMAIN_ID=4`**. Any machine that should see the robot must use it |
| DDS | Fast DDS (`rmw_fastrtps_cpp`). Use UDP-only transport for Nav2 (see §8) |

**Our code never sends UDP to the motion host directly.** `lite3_domain_bridge_4` already
does this. A second UDP sender would give the robot two conflicting command streams. The UDP
protocol is documented in §6.4 so you understand what the bridge does, and in case the
bridge is ever unavailable.

### 1.1 Hardware specifications

Sources: **[doc]** manufacturer guides (above), **[datasheet]** vendor spec pages
([Mid-360](https://www.livoxtech.com/mid-360/specs),
[Gemini 335](https://www.orbbec.com/products/stereo-vision-camera/gemini-335/), read
2026-11-13), **[live]** measured on our robot, **[derived]** computed from the others.

**Robot (Lite3)**

| Item | Value |
|---|---|
| Body | 548 × 370 mm; body 11 kg (with battery and HipX joints) + 4 × 1.12 kg legs ≈ 15.5 kg [doc]. More geometry in §7 |
| Standing height | `base_link` ≈ 0.35 m above the floor; lying down ≈ 0.07–0.11 m [doc][live] |
| Motion host | Rockchip **RK3588** (ARM), runs the locomotion firmware [doc] |
| Joints | 12 (3 per leg: HipX, HipY, Knee). High-power DC motor + reducer + absolute encoder each [doc] |
| Joint limits | HipX −24…24°, 6.48 N·m nominal / 24 N·m max, 26.2 rad/s. HipY −200…20°, 6.48 / 24 N·m, 26.1 rad/s. Knee 34.5…156°, 9.72 / 36 N·m, 17.1 rad/s [doc] |
| Body IMU | 5 mm ahead of the body origin, on the motion host [doc] |
| Speed limits | ±1.0 m/s forward, ±0.5 m/s sideways, ±1.5 rad/s turn (§6.2) [doc]; team limit 0.25 m/s, 0.5 rad/s |
| Protections | Overtemperature → stops and lies down; fall prediction → joints damped, speed to zero (§6.2) [doc] |
| Power for accessories | 24 V port on the function box (lidar via XT30) [doc] |
| Perception host (manufacturer) | Jetson Xavier NX in the Pro/LiDAR edition; the kit guide uses an AGX Orin 64 GB. Not used by us [doc] |

**Robot computer (where our code runs)** [live]

| Item | Value |
|---|---|
| Type | LXC container on a Proxmox host (the bridge runs in a sibling guest, §1) |
| CPU | Intel Core i5-13420H, 10 threads visible |
| RAM | 6 GB |
| Disk | 30 GB, ~6.7 GB free on 2026-11-13 (a full recording uses ~1.6 GB/min, so ~4 min) |
| GPU | **None usable** (Intel iGPU only, no CUDA). YOLO and other networks run on the CPU |
| OS | Ubuntu 22.04.5, ROS 2 Humble |
| Network | `eth0` 192.168.1.74 (robot), `eth1` 192.168.10.74 (lab) |

**Lidar: Livox Mid-360**

| Item | Value |
|---|---|
| Mount | 186.78 mm ahead, 124.98 mm above `base_link`, pitched 15° down (§5) [doc][live]. ≈ 0.475 m above the floor when standing [derived] |
| Range | 40 m @ 10 % reflectivity, 70 m @ 80 %. Blind zone 0.1 m (0.1–0.2 m recorded but precision not guaranteed) [datasheet] |
| FOV | 360° horizontal, **−7° to +52°** vertical [datasheet] |
| Points | 200,000 pts/s, 10 Hz frames (≈ 20,000 points per message), non-repetitive scan [datasheet][live] |
| Precision | ≤ 2 cm @ 10 m, ≤ 3 cm @ 0.2 m (1σ); angular < 0.15° [datasheet] |
| Built-in IMU | ICM40609 (`/livox/imu`, acceleration in g) [datasheet][live] |
| Other | 905 nm, Class 1 eye-safe; 6.5 W average; 9–27 V; 100BASE-TX Ethernet; PTPv2/GPS sync; IP67; −20…55 °C; 265 g [datasheet] |

What the mount means in practice [derived, checked live 2026-11-13]:
- **Ahead:** the 15° pitch turns the −7…+52° FOV into **−22…+37°**. The lowest ray reaches
  the floor about 1.2 m ahead of the lidar (≈ 1.36 m from `base_link`); live, the first floor
  returns were at ≈ 1.25 m. Low objects closer than that are not seen.
- **Behind:** the FOV is −7+15 = **+8…+67°**, so it never sees the floor behind the robot.
- **Sides:** about −7…+52°, floor from ≈ 3.9 m out.
- About 36 hits per scan land on the robot itself (§3.2).

**Camera: Orbbec Gemini 335** (the guides describe a RealSense D435i; see §1)

| Item | Value |
|---|---|
| Mount | 252.28 mm ahead, 74.10 mm above `base_link`, pitched 20° down (§5) [doc][live]. ≈ 0.42 m above the floor when standing [derived] |
| Depth | Active/passive stereo, 850 nm. Range 0.10–20 m+, **optimal 0.26–3 m**. Precision ≤ 1.5 % at 2 m [datasheet] |
| Depth FOV | 90° × 65° (H × V); up to 1280×800 @ 30 fps [datasheet] |
| Colour | Up to 1920×1080 @ 30 fps; FOV 86° × 55° [datasheet] |
| **Our streams** | Colour 640×480 `rgb8` and depth 640×480 `16UC1` (mm), both 30 Hz, not aligned (§3.1) [live] |
| IMU | Supported by the camera [datasheet]; not published on our robot (no camera IMU topic seen) [live] |
| Other | USB 3.0 Type-C (data and power); < 3.0 W; −10…45 °C, indoor/outdoor; IP5X; 90 × 25 × 30 mm, 97 g [datasheet] |

What the mount means [derived]: the depth image covers 52.5° down to 12.5° up from
horizontal, so the floor shows from ≈ 0.3 m ahead of the camera. Colour covers 47.5° down to
7.5° up, so the floor shows from ≈ 0.4 m. The top edge of the depth image is at about
0.42 + 0.22 × d m above the floor at distance d (≈ 0.64 m at 1 m, ≈ 0.86 m at 2 m), so a
table top (~0.75 m) only comes into view beyond ≈ 1.5 m. Unlike the lidar, the camera sees
low objects close in front, but only forward.

**Other sensors** [live]: joint encoders (`/joint_states`), leg odometry (`/leg_odom*`), rear
ultrasound (`/us_publisher/ultrasound_distance`), body IMU (`/imu/data`). See §3.3.

---

## 2. Dependencies

### 2.1 Packages you need to build against the robot interface

| Package | Needed for | Where to get it |
|---|---|---|
| ROS 2 Humble (`ros-humble-desktop`) | everything | apt |
| `transfer_interfaces` | `MotionSimpleCMD`, `MotionComplexCMD` (mode/gait commands) | `github.com/DeepRoboticsLab/Lite3_ROS`, branch `ros2-foxy`, `src/transfer_interfaces`. On the robot: `~/lite3_ws/src/transfer_interfaces`. Or recreate it from §2.2 |
| `livox_ros_driver2` | only for `/livox/lidar` (`CustomMsg`); `/livox/points` is a standard `PointCloud2` | Livox-SDK `livox_ros_driver2`. On the robot: `~/lite3_ws/src/livox_ros_driver2`. Message definitions in §2.2 |
| `orbbec_camera_msgs` | only for the `/camera/*/metadata` and status topics (rarely needed) | Orbbec ROS 2 SDK |
| `fast_lio` | lidar-inertial odometry (`/odom`) | `github.com/hku-mars/FAST_LIO`, branch `ROS2`. On the robot: `~/ros2_ws/src/FAST_LIO` |
| `lite3_launcher` | lab launch files: Fast-LIO + `odom_relay` + `/scan`, SLAM, Nav2, twist_mux | **Robot only, not in git**: `~/ros2_ws/src/lite3_launcher`. Copy it off the robot if you need it |
| Nav2, slam_toolbox, pointcloud_to_laserscan, twist_mux, cv_bridge, tf2_geometry_msgs, image_transport_plugins | navigation/perception | apt `ros-humble-*` (full list in `agent.md` §2) |

Order for sourcing on the robot (already in `~/.bashrc`):
```bash
source /opt/ros/humble/setup.bash
source ~/lite3_ws/install/local_setup.bash   # transfer_interfaces, livox_ros_driver2
source ~/ros2_ws/install/local_setup.bash    # lite3_launcher, fast_lio
source ~/challenge_ws/install/setup.bash
export ROS_DOMAIN_ID=4
```

### 2.2 Custom message definitions (enough to recreate the packages offline)

`transfer_interfaces/msg/MotionSimpleCMD.msg`
```
int32 cmd_code
int32 size
int32 type
```

`transfer_interfaces/msg/MotionComplexCMD.msg`
```
int32 cmd_code
int32 size
int32 type
float64 data
```

`livox_ros_driver2/msg/CustomMsg.msg`
```
std_msgs/Header header
uint64 timebase          # time of first point (ns)
uint32 point_num
uint8  lidar_id
uint8[3] rsvd
CustomPoint[] points
```
`livox_ros_driver2/msg/CustomPoint.msg`
```
uint32 offset_time       # ns, relative to timebase
float32 x                # m
float32 y
float32 z
uint8 reflectivity       # 0–255
uint8 tag
uint8 line               # laser number
```

To build `transfer_interfaces` yourself, make an `ament_cmake` package with
`rosidl_generate_interfaces(${PROJECT_NAME} "msg/MotionSimpleCMD.msg" "msg/MotionComplexCMD.msg")`.
Keep the package name and field names exactly as above, or DDS type matching with the robot fails.

---

## 3. Topics provided by the robot (`lite3_domain_bridge_4`)

These are always present while the robot is on, even if none of our software runs. Rates
are nominal: from the drivers' configuration and earlier measurements, not guaranteed.

### 3.1 Camera (Orbbec RGB-D)

| Topic | Type | Rate | Content [live] |
|---|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 30 Hz | 640×480, `rgb8`, frame `camera_color_optical_frame`. About 0.9 MB per frame; avoid it over Wi-Fi |
| `/camera/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | 30 Hz | JPEG, about 75 KB per frame. **Use this for perception and recording** |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | `plumb_bob`. K = [461.92, 0, 319.89; 0, 461.95, 240.90; 0, 0, 1]. D = [0.0062, −0.0475, 0.0006, 0.0002, 0.0324] |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 30 Hz | 640×480, `16UC1`, **millimetres**, 0 = no data. Frame `camera_depth_optical_frame`. **Not aligned to colour** |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | K = [372.79, 0, 319.40; 0, 372.79, 241.50; 0, 0, 1], D = 0 |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | ~3–30 Hz | About 256k points, fields x, y, z (float32), `point_step` 16. Very large; prefer rebuilding from depth |
| `/camera/color/metadata`, `/camera/depth/metadata` | `orbbec_camera_msgs/Metadata` | | Exposure etc. |
| `/camera/device_status`, `/camera/depth_filters/status` | `orbbec_camera_msgs/...` | | Driver health |
| `/image_raw`, `/camera_info` | `Image`, `CameraInfo` | | Second camera. Advertised, but **no messages were seen** on 2026-11-13 |

**Depth alignment:** depth and colour use different intrinsics (fx 373 vs 462) and are
about 14 mm apart (`camera_depth_frame → camera_color_frame` in §5). To get the depth of a
colour pixel, either reproject through TF and both `camera_info`s, or have the camera driver
turn on depth-to-colour registration (`depth_registration:=true` in the camera launch inside `~/.config/tmuxinator/lite3.yml` on the system computer; not done yet; there is no `/camera/aligned_depth_to_color/*` topic).

### 3.2 Lidar (Livox Mid-360)

| Topic | Type | Rate | Content |
|---|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | 10 Hz | About 20,000 points per message, frame `livox_frame`. Fast-LIO input [live] |
| `/livox/points` | `sensor_msgs/PointCloud2` | 10 Hz | Same points, standard cloud. Fields (offset, type): `x` 0 f32, `y` 4 f32, `z` 8 f32, `t` 12 u32 (ns offset), `intensity` 16 f32, `tag` 20 u8, `line` 21 u8; `point_step` 22, unorganised (`height` 1) [live] |
| `/livox/imu` | `sensor_msgs/Imu` | ~200 Hz | Lidar's built-in IMU, frame `livox_frame`. Acceleration in **g** (z ≈ 0.97 at rest), not m/s² [live] |

The lidar is mounted pitched **15° nose-down** (see TF in §5). Points within about 0.4–0.5 m
are mostly the robot's own body and legs; filter them.

Measured 2026-11-13 [live]:
- **Floor height in `base_link`** (from `/livox/points` through the static mount): about
  **−0.35 m when standing**, about −0.07 to −0.11 m when lying down. A quick way to tell
  whether the robot is standing, since the bridge doesn't publish the basic state. The floor
  appears to drop ~3 cm per metre ahead (≈2° tilt of the robot or mount).
- About 36 self-hits per scan near x ≈ 0, ~0.2 m above `base_link`. No floor returns closer
  than ~1.25 m ahead (blind zone).
- **Latency** (arrival time minus header stamp): `/livox/points` ≈ 0.14 s median (stamp is
  the scan start), `/livox/imu` ≈ 2 ms. While `ros2 bag record` of
  `/camera/depth/image_raw` ran, `/livox/points` arrived ≈ 1.2 s late, enough to make Nav2's
  collision_monitor (0.5 s timeout) ignore it.

### 3.3 Body state from the motion host

| Topic | Type | Rate | Content |
|---|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | ~200 Hz | Body IMU, frame `base_link`. Orientation is a quaternion built from roll/pitch/yaw; angular velocity in rad/s; acceleration in m/s² (z ≈ 9.8 at rest). Covariances are all 0 (unset) [live][doc] |
| `/leg_odom` | `geometry_msgs/PoseWithCovarianceStamped` | ~50 Hz | Leg odometry pose, frame `odom`. Orientation is **yaw only**. Covariance unset [live][doc]. **`position.z` is probably yaw in rad, not height:** the bridge copies `pos_world[2]` into z, and the interface document defines `pos_world` as {x, y, yaw}. Ignore z. The Perception guide lists this topic as `nav_msgs/Odometry`; the live type is `PoseWithCovarianceStamped` |
| `/leg_odom2` | `nav_msgs/Odometry` | same | Same pose plus body-frame velocity: `twist.linear.x/y` (m/s), `twist.angular.z` (rad/s). **`frame_id` and `child_frame_id` are empty** [live] |
| `/joint_states` | `sensor_msgs/JointState` | ~100 Hz | 12 joints, **position only** (no velocity/effort). Names below [live] |
| `/us_publisher/ultrasound_distance` | `std_msgs/Float64` | ~50 Hz | Ultrasound distance in m, clamped to [0.28, 4.50]. The source publishes `ultrasound[1]`, which the interface document defines as `backward_distance`, so this is the **rear** sensor [doc]. 0.28 means "0.28 m or closer" (also the value seen at rest on 2026-11-13). Front distance isn't published |
| `/handle_state` | `geometry_msgs/Twist` | ~50 Hz | Handheld controller sticks, normalised −1…1: `linear.x` = left stick forward, `linear.y` = left stick sideways, `angular.z` = −(right stick yaw). These are stick positions, not velocities. QoS **BEST_EFFORT** [live][doc] |
| `/commands/joy_raw` | `sensor_msgs/Joy` | on input | App/joystick passthrough. Nothing seen while idle |
| `/sensor_status/{imu,joint,odom,odom2,ultrasound}_isalive` | `std_msgs/Bool` | 2 Hz | Driver health flags [live] |
| `/tf_static` | `tf2_msgs/TFMessage` | latched | Sensor mounts (§5) |

Joint order and names in `/joint_states` (signs are negated relative to the raw motion-host
values) [doc]:

| Index | Name | Joint | | Index | Name | Joint |
|---|---|---|---|---|---|---|
| 0 | `LF_Joint` | front-left HipX (ab/adduction) | | 6 | `LB_Joint` | hind-left HipX |
| 1 | `LF_Joint_1` | front-left HipY (flexion) | | 7 | `LB_Joint_1` | hind-left HipY |
| 2 | `LF_Joint_2` | front-left knee | | 8 | `LB_Joint_2` | hind-left knee |
| 3 | `RF_Joint` | front-right HipX | | 9 | `RB_Joint` | hind-right HipX |
| 4 | `RF_Joint_1` | front-right HipY | | 10 | `RB_Joint_1` | hind-right HipY |
| 5 | `RF_Joint_2` | front-right knee | | 11 | `RB_Joint_2` | hind-right knee |

**Not available through the bridge:** gait state, basic state (sitting/standing/fallen),
control mode (auto/manual), and battery level. The motion host sends these in its UDP state
packet (§6.4.3), but the bridge doesn't publish them. You can't confirm a mode or gait change
from ROS; watch the robot or the handheld controller.

### 3.4 Timestamps (important for synchronisation)

| Stamped with | Topics |
|---|---|
| **Wall clock (Unix epoch)** | all `/camera/*`, `/livox/*`, `/tf_static` |
| **Steady clock (seconds since motion-host/computer boot)** | `/imu/data`, `/leg_odom`, `/leg_odom2`, `/joint_states` |

The difference is about 1.79 × 10⁹ s [live]. Don't feed body IMU or leg odometry stamps into
TF lookups or message synchronisers together with lidar/camera data. Use the time of receipt,
or a measured offset.

### 3.5 QoS of robot-published topics

The bridge publishes with **RELIABLE, VOLATILE** (except `/handle_state`: BEST_EFFORT). A
subscriber with either `qos_profile_sensor_data` (best effort) or the default profile
connects. Subscribe to `/tf_static` with TRANSIENT_LOCAL (the `tf2_ros` listener already does).

---

## 4. Topics from the localisation/navigation stack (only when it's running)

These exist only when someone has started `lite3_launcher` / `navigation` / `lite3-maze`.
Check `ros2 node list` first.

| Topic | Type | Publisher | Notes |
|---|---|---|---|
| `/fastlio/odometry` | `nav_msgs/Odometry` | Fast-LIO (`laser_mapping`) | Lidar-frame odometry, about 10 Hz |
| `/odom` + TF `odom → base_link` | `nav_msgs/Odometry` | `odom_relay` (`lite3_launcher`) | Fast-LIO converted to `base_link`. **The only source of `odom → base_link`**; leg odometry is not in TF |
| `/cloud_registered_body` | `PointCloud2` | Fast-LIO | De-skewed scan in the body frame |
| `/scan` | `sensor_msgs/LaserScan` | `pointcloud_to_laserscan` | Points 0.1–0.5 m above `base_link` (min_height still marked TODO), 0.4–20 m range, 0.5° steps, full 360° |
| `/map` + TF `map → odom` | `nav_msgs/OccupancyGrid` (reliable, transient local) | `slam_toolbox` | |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 `bt_navigator` | Send goals with `frame_id: map` |
| `/cmd_vel_nav` → `/nav_cmd_vel` | `geometry_msgs/Twist` | Nav2 controller → velocity smoother | Must still be forwarded to `/cmd_vel` (§6.2) |
| `/joy_cmd_vel` | `Twist` | `teleop_twist_joy` (USB gamepad on the robot computer; LB = enable, RB = turbo) | Lab launcher only |
| `/robot_mode`, `/maze/*`, `/n_humans` | | `lite3-maze` | See `src/navigation/lite3-maze/README.md` |
| `/qr/detections`, `/humans/detections`, `/posters/detections`, `/found` | `std_msgs/String` | `team_perception` | See `src/challenge_files/onboard-external_communication.md` |

Lab `twist_mux` (`lite3_launcher/config/twist_mux_topics.yaml`): `joy_cmd_vel` (priority
100, timeout 0.5 s) beats `nav_cmd_vel` (priority 10, timeout 0.5 s), output `/cmd_vel`.
Its output is VOLATILE; see the QoS rule in §6.2.

---

## 5. Coordinate frames and TF

```
map ──(slam_toolbox)──► odom ──(odom_relay / Fast-LIO)──► base_link ─┬─(static)─► livox_frame
                          └──(static)──► lidar_init                    └─(static)─► camera_link ─► camera_depth_frame ─┬─► camera_depth_optical_frame
                                                                                                                      └─► camera_color_frame ─► camera_color_optical_frame
```

Static transforms [live, 2026-11-13] (translation in m, quaternion x y z w):

| Parent → child | Translation | Rotation | Meaning |
|---|---|---|---|
| `base_link → livox_frame` | (0.18678, 0, 0.12498) | (0, 0.1305, 0, 0.9914) | Lidar, pitched **15° down** |
| `base_link → camera_link` | (0.25228, −0.00050, 0.07410) | (0, 0.1736, 0, 0.9848) | Camera, pitched **20° down** |
| `camera_link → camera_depth_frame` | (0, 0, 0) | identity | |
| `camera_depth_frame → camera_color_frame` | (0.00184, −0.01409, 0.00013) | ≈ identity | Colour sensor about 14 mm to the right of depth |
| `camera_*_frame → camera_*_optical_frame` | (0, 0, 0) | (−0.5, 0.5, −0.5, 0.5) | Standard optical frame (z forward, x right, y down) |
| `odom → lidar_init` | same as `base_link → livox_frame` | | Fast-LIO's start frame |

`base_link` follows ROS REP-103: **x forward, y left, z up**. Without a running Fast-LIO stack,
`/tf` is empty and only `/tf_static` exists.

---

## 6. Controlling motion

### 6.1 The control path

1. A human stands the robot up with the handheld controller.
2. The robot must be in **auto (navigation) mode** to accept `/cmd_vel`. A human operator
   sends this (`lite3_auto`, §6.3). **Our code should not switch to auto mode on its own.**
3. Your node publishes `geometry_msgs/Twist` on `/cmd_vel`.
4. `lite3_domain_bridge_4` converts each Twist into three UDP speed commands (§6.4).
5. To hand control back: `lite3_manual`. The handheld controller drives again and `/cmd_vel` is ignored.

### 6.2 `/cmd_vel`: velocity commands

Type `geometry_msgs/msg/Twist`. Use ROS-standard signs; the bridge converts them.

| Field | Unit | Sign (ROS standard) | Robot limit [doc] | Team limit (so far) | UDP command |
|---|---|---|---|---|---|
| `linear.x` | m/s | + forward | ±1.0 | ≤ 0.25 (0.10 for first trials) | `0x0140` (320) |
| `linear.y` | m/s | + left [doc] | ±0.5 | keep small; check once on hardware | `0x0145` (325), passed unchanged |
| `angular.z` | rad/s | + counter-clockwise (left turn) | ±1.5 | ≤ 0.5 | `0x0141` (321), **sign flipped by the bridge** |
| all others | | ignored | | | |

- **Never negate `angular.z` yourself.** The bridge already does it.
- **Signs:** the Perception Development Guide (§5.2) documents the ROS `/cmd_vel` convention for
  this bridge: `linear.y` + = left, `angular.z` + = turn left. Only the *joystick axis* commands
  (a different, raw interface) use "positive = right". Nobody has driven sideways on our robot
  yet, so do one 0.05 m/s check before relying on it.
- **Rate:** publish continuously while moving. The manufacturer's own example uses 10 Hz
  (`ros2 topic pub -r 10`); we use 20–50 Hz (`lite3-maze` uses 50 Hz).
- **Timeout: none is documented for velocity commands.** The 250 ms "stop if nothing
  arrives" rule in the interface document applies to joystick *axis* commands only. No timeout
  is documented for the speed commands the bridge sends, and the manufacturer bridge adds
  none. **Assume the robot keeps walking at the last commanded speed if your node dies.**
  Always send an explicit zero Twist to stop, and keep the handheld STOP ready.
- **QoS:** publish **RELIABLE + TRANSIENT_LOCAL, depth 1, lifespan about 0.25–0.5 s**.
  - On 2026-11-13 the bridge's subscription asked for RELIABLE + VOLATILE. On 2026-11-12 it
    asked for RELIABLE + TRANSIENT_LOCAL, and it created its subscription only after it saw a
    publisher. A TRANSIENT_LOCAL publisher connects in both cases, and also to the manufacturer
    bridge's BEST_EFFORT subscriber. A VOLATILE publisher (Nav2, twist_mux, `ros2 topic pub` with
    default QoS) does **not** connect to a TRANSIENT_LOCAL subscriber.
  - The lifespan stops an old latched command from being replayed to a subscriber that joins late.
  - Check what the bridge currently asks for: `ros2 topic info -v /cmd_vel`.
  - Reference implementation: `src/archive/navigation/cmd_vel_bridge.py` (Nav2 `/nav_cmd_vel` → `/cmd_vel` with this QoS and clamping).

**Safety rules for any node that publishes `/cmd_vel`:**
1. Only one publisher at a time. Check `ros2 topic info -v /cmd_vel` before you drive.
2. Clamp speeds in your node; don't trust the caller.
3. If your input goes quiet (planner stall, lost sensor), send **one** zero Twist and then
   **stop publishing**, so teleop or another node can take over.
4. Send a zero Twist on shutdown (and `lite3_manual` if your run entered auto mode).
5. Someone holding the handheld controller (with its STOP button) stays next to the robot
   during every autonomous trial.

Built-in firmware protections [doc]: **overtemperature** makes the robot stop and lie down
(let it cool before reuse). **Fall protection** stiffens the joints and zeroes the velocity
when the IMU predicts a fall. After either one, a human must clear the area and stand the
robot up again. Code can't detect these events through ROS.

Minimal compliant publisher (Python, rclpy):
```python
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist

CMD_QOS = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     lifespan=Duration(seconds=0.3))
MAX_V, MAX_W = 0.25, 0.5

class Driver(Node):
    def __init__(self):
        super().__init__('my_driver')
        self.pub = self.create_publisher(Twist, '/cmd_vel', CMD_QOS)
        self.target = Twist()                # set by your logic
        self.last_update = self.get_clock().now()
        self.active = False
        self.create_timer(0.02, self.tick)    # 50 Hz

    def tick(self):
        stale = (self.get_clock().now() - self.last_update) > Duration(seconds=0.5)
        if stale:
            if self.active:                  # one stop, then go quiet
                self.pub.publish(Twist()); self.active = False
            return
        t = Twist()
        t.linear.x = max(-MAX_V, min(MAX_V, self.target.linear.x))
        t.angular.z = max(-MAX_W, min(MAX_W, self.target.angular.z))
        self.pub.publish(t); self.active = True

def main():
    rclpy.init(); n = Driver()
    try: rclpy.spin(n)
    finally:
        n.pub.publish(Twist()); rclpy.shutdown()
```

Manual test (robot standing, in auto mode, area clear):
```bash
ros2 topic pub --once --qos-reliability reliable --qos-durability transient_local \
  /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'
```

### 6.3 `/simple_cmd`: modes, gaits and stop

Type `transfer_interfaces/msg/MotionSimpleCMD`. Set `cmd_code`, `size: 0`, `type: 0`. Same QoS
as `/cmd_vel` (reliable + transient local, depth 1, short lifespan). The bridge sends the
three fields as a 12-byte UDP packet. Send each command **once**; these aren't streams.

Shell aliases on the robot (from `~/.bashrc`, the block marked `#DO NOT CHANGE`, which is the one in effect):
```bash
alias lite3_auto='ros2 topic pub /simple_cmd transfer_interfaces/msg/MotionSimpleCMD "{cmd_code: 0x21010C03, type: 0}" --once'
alias lite3_manual='ros2 topic pub /simple_cmd transfer_interfaces/msg/MotionSimpleCMD "{cmd_code: 0x21010C02, type: 0}" --once'
```
**Why it prints `Waiting for at least 1 matching subscription(s)...`:** `--once` waits for a subscriber
so the single message isn't lost. `lite3_domain_bridge_4` creates its `/simple_cmd` (and `/cmd_vel`)
subscription **only after it sees a publisher** (RELIABLE + VOLATILE, so default-QoS `ros2 topic pub` matches).
With no recent publisher, `/simple_cmd` doesn't exist at all ("Unknown topic") and the first
`lite3_auto` waits until the bridge reacts; once the subscription exists, later publishers match at once
[live, 2026-11-13]. Don't use `-w 0`: the command would be sent before anyone listens and be lost.

Codes relevant to us [doc]:

| Code | Name | When valid | Our use |
|---|---|---|---|
| `0x21010C03` | **Navigation (auto) mode**: follow `/cmd_vel` | Not in AI state | `lite3_auto`; sent by the human operator |
| `0x21010C02` | **Manual mode**: handheld controller drives | | `lite3_manual`; safe fallback, send on shutdown |
| `0x21020C0E` | **Software emergency STOP** | Any | Used in the 2026-11-12 trial; delivery confirmed. Robot needs operator recovery afterwards |
| `0x21010202` | Stand / sit toggle | | **Don't send from code**; the operator handles it |
| `0x21010D06` | Move mode (needed before gait changes) | Not in AI state | `lite3-maze` sends it before gait requests |
| `0x21010D05` | Pose mode (body posture via joystick axes) | Not in AI state | Not used |
| `0x21010401` | RUG gait, General gear (state 2) | Move mode, not AI | Normal rough-ground gait |
| `0x21010407` | RUG gait, **H-Step** (high stepping, state 13) | Move mode, not AI | For stairs/obstacles. **On 2026-11-12 the robot became unsteady after this was sent; cause not found.** Only with the operator's agreement |
| `0x21010402` | RUG gait, Grip gear (state 6) | Move mode, not AI | Not tested |
| `0x21010300` / `0x21010307` / `0x21010303` | Flat gait slow / medium / fast (states 0 / 4 / 5) | Move mode, not AI | Not tested |
| `0x21010406` | Flat crawl gait (toggle) | Move mode, not AI | Not tested |
| `0x21010C06` | Keep stepping (value −1 enable, 2 disable; needs `size` = value) | Not AI | Not tested |
| `0x21040001` | Heartbeat (≥ 2 Hz for a direct UDP client) | | The manufacturer's ROS bridge (`Jetson2Motion.cpp`) **never sends it**, and the guide's `/cmd_vel` workflow still works, so the ROS path doesn't need it. Don't send it from our code [doc/source] |
| `0x21012109` | Motion-host AI options: value `0x00` all off, `0x20` auto-stop, `0xC0` people tracking | Not AI | **Don't send.** Tracking would make the robot follow people. Current setting on our robot is unknown |
| `0x21010C0B` | Stop a continuous action (Twist, Moonwalk); send value 0 **and** 1 | | Only needed if someone starts a show action |
| `0x21010C05` | Reset joints to zero | | **Don't send** |
| `0x21010528` / `0x2101052B` | Enter / exit AI state | | **Don't send.** Auto mode doesn't work in AI state |
| `0x21010C01` | Save fault data, then soft e-stop and quit the motion program | | **Never send** |

Leave out everything else from the manufacturer list (backflip, jumps, moonwalk, voice
commands, AI gaits). Some of these are dangerous in the maze. **Never enable SDK mode**
(DEEP Robotics app slider): it replaces the factory locomotion controller with a user
policy, resets the joints, and disables everything in this document.

Normal sequence: operator stands the robot → (optional, agreed) gait request → operator
sends `lite3_auto` → our node drives on `/cmd_vel` → `lite3_manual` at the end. To stop:
zero Twist, then `lite3_manual`. For an emergency: handheld STOP, or `0x21020C0E`.

Only `/cmd_vel` and `/simple_cmd` are forwarded. **`/complex_cmd` and `/cmd_vel_corrected` from
the manufacturer bridge are not subscribed by the group bridge** [live].

### 6.4 UDP protocol (what the bridge sends; reference only)

All values are little-endian. The target is `192.168.1.120:43893`. The manufacturer bridge
listens for motion-host reports on local UDP port `43897`.

#### 6.4.1 Simple command, 12 bytes

| Offset | Type | Field |
|---|---|---|
| 0 | int32 | `cmd_code` |
| 4 | int32 | `size` (command value; 0 if unused) |
| 8 | int32 | `type` = 0 |

Python: `struct.pack('<iii', code, value, 0)`.

#### 6.4.2 Speed command (complex), 20 bytes

| Offset | Type | Field |
|---|---|---|
| 0 | int32 | `cmd_code`: `0x0140` vx, `0x0141` yaw rate, `0x0145` vy |
| 4 | int32 | `size` = 8 (bytes of data) |
| 8 | int32 | `type` = 1 |
| 12 | float64 | value (m/s or rad/s) |

Python: `struct.pack('<iiid', 0x0140, 8, 1, vx)`. For each `/cmd_vel` message, the bridge sends
vx, then vy, then `−angular.z`. The robot only accepts these in navigation mode.

#### 6.4.3 Reports from the motion host (what the bridge turns into topics)

The header is the same 12 bytes (`code`, `size`, `type`); data follows. The manufacturer
bridge tells packet types apart by **total length** and code.

| Code | Rate | Packet size | Content | Becomes |
|---|---|---|---|---|
| `0x0901` (2305) | 50 Hz | 220 B | `RobotStateUpload` (offsets below) | `/leg_odom`, `/leg_odom2`, ultrasound |
| `0x0902` (2306) | 100 Hz | 108 B | `double joint_angle[12]` rad, order FL, FR, HL, HR × (HipX, HipY, Knee) | `/joint_states` |
| `0x0903` | 100 Hz | 108 B | `double joint_vel[12]` rad/s | not published |
| `0x0905` (2309) | | 60 B | Handle state: `double left_axis_forward, left_axis_side, right_axis_yaw, goal_vel_forward, goal_vel_side, goal_vel_yaw` | `/handle_state` |
| `0x010901` | ~200 Hz | 52 B | IMU: `uint32 timestamp; float roll, pitch, yaw (deg), ωroll, ωpitch, ωyaw (rad/s), ax, ay, az`. Not in the interface document; from the bridge source | `/imu/data` |

`RobotStateUpload` byte offsets, counted from the start of the data (add 12 for the header).
These were compiled from the manufacturer's `protocol.hpp`, and the document's struct matches:

| Offset | Field | | Offset | Field |
|---|---|---|---|---|
| 0 | `int robot_basic_state` | | 160 | `unsigned touch_down_and_stair_trot` (unused) |
| 4 | `int robot_gait_state` | | 164 | `bool is_charging` (unused) |
| 8 | `int robot_policy_state` (AI gait) | | 168 | `unsigned error_state` (unused) |
| 16 | `double rpy[3]` (deg) | | 172 | `int robot_motion_state` |
| 40 | `double rpy_vel[3]` (rad/s) | | 176 | `double battery_level` (fraction) |
| 64 | `double xyz_acc[3]` (m/s²) | | 184 | `int task_state` (unused) |
| 88 | `double pos_world[3]` = {x, y, yaw rad} | | 188–191 | `bool is_robot_need_move, zero_position_flag, is_after_first_start, is_voice_ctrl_enable` |
| 112 | `double vel_world[3]` = {vx, vy, ωz} | | 192 | `double ultrasound[2]` = {front, rear} m |
| 136 | `double vel_body[3]` = {vx, vy, ωz} | | | |

`robot_basic_state` values: 1 sitting, 4 prepare, 5 sit→stand, **6 standing (torque control)**,
7 stand→sit, **8 lose-control protection**, 9 posture adjustment, 11 flipping over, 16 AI,
17 reset to zero. `robot_gait_state`: 0 flat slow, 2 RUG general, 4 flat medium, 5 flat fast,
6 RUG grip, 12 moonwalk, 13 RUG H-Step. `robot_motion_state`: 0 standing still, 1 stepping.

`protocol.hpp` leaves out `is_after_first_start` and `is_voice_ctrl_enable`, but they sit in
alignment padding, so both layouts are 220 bytes and every other offset is the same.

---

## 7. Robot geometry for planning

| Quantity | Value | Source |
|---|---|---|
| Body length × width | 0.548 × 0.370 m | manufacturer [doc] |
| Lab Nav2 footprint | rectangle ±0.305 × ±0.185 m (0.61 × 0.37) | `lite3_launcher/config/nav2_params.yaml` |
| `lite3-maze` footprint (allows for moving feet) | 0.64 × 0.46 m | `lite3-maze/src/lite3_maze/config/robot.yaml` |
| Max traversable step used by `lite3-maze` | 0.16 m | same file [unverified on hardware] |
| `base_link` height | about 0.35 m | manufacturer source |
| Body origin | geometric centre of the body (= `base_link`). Top of body +0.074 m, bottom −0.044 m | Motion guide [doc] |
| Body IMU | (0.005, 0, 0) m from the body origin | Motion guide [doc] |
| Hip (HipX) joints | x ±0.1745 m, y ±0.062 m; HipY a further ±0.0985 m outward | Motion guide [doc] |
| Leg centre spacing | 0.328 m left–right (feet at about y = ±0.164 m, plus 0.02 m foot radius); 0.349 m front–hind hips | Motion guide [doc] |
| Mass | body 11 kg + 4 × 1.12 kg legs ≈ 15.5 kg | Motion guide [doc] |
| Joint ranges (raw motion-host sign) | HipX −24…24°, HipY −200…20°, Knee 34.5…156°. `/joint_states` negates these | Motion guide [doc] |

When standing, the feet stay roughly inside the body outline (±0.164 m wide vs ±0.185 m
body). The swing envelope during RUG / H-Step gaits has **not been measured**, so add margin.

---

## 8. DDS and network settings

- Use `ROS_DOMAIN_ID=4` for the robot. For tests that must not reach the robot, use
  `ROS_DOMAIN_ID=89` (or anything other than 4) and `ROS_LOCALHOST_ONLY=1`.
- Fast DDS with **UDP-only transport** is needed for Nav2 on the robot (with shared memory its
  lifecycle startup stalled):
  `export FASTRTPS_DEFAULT_PROFILES_FILE=~/challenge_ws/src/archive/config/fastdds_udp.xml`
- The robot computer has no screen. Run RViz/rqt on a laptop on the lab network
  (192.168.10.x) with the same domain ID.
- Don't start the manufacturer `transfer` launch (`ros2 launch transfer transfer_launch.py`)
  while `lite3_domain_bridge_4` is running.
- Don't start a second SLAM/Nav2/maze stack while one is already running. Check
  `ros2 node list` and `ps aux | grep -E "slam|nav2|maze|fastlio"`.

---

## 9. Developing without the robot

| Resource | What it gives you |
|---|---|
| `ROS_bags/teleop_run1` (1.5 GB, 2026-11-12, about 52 s) | Real camera (compressed colour + depth + infos), Livox, IMUs, leg odometry, `/tf_static`. No dynamic `/tf`. Play it with `ros2 bag play --clock` and run Fast-LIO yourself |
| `recordings/maze_*`, `recordings/integrated_maze_*` | Logs, saved maps (`maze_map.pgm/.yaml`), QR evidence from the 2026-11-12 trials |
| `src/archive/test/fake_lite3.py` | Simulated room: publishes `/scan`, `/odom`, TF and accepts `/cmd_vel`. Use on domain 89 |
| `src/navigation/lite3-maze` MuJoCo simulator (`./scripts/run_sim.sh`) | Full maze simulation with RViz on domain 31. Accepts `/cmd_vel` and `/robot_mode` |
| §2.2 message definitions | Recreate `transfer_interfaces` / `livox_ros_driver2` messages so your code builds |

To test command code offline, run a stand-in subscriber on another domain that logs `/cmd_vel`
and `/simple_cmd` with RELIABLE + VOLATILE QoS (what the bridge uses today), and check rate,
clamping and the single stop message.

---

## 10. Open questions (please fill in if you find out)

Answered from the manufacturer guides (2026-11-13): `linear.y` sign (+ = left), heartbeat
(not needed on the ROS path), ultrasound topic (rear sensor), `RobotStateUpload` layout.

Still open:
1. ~~Has `/cmd_vel` through `lite3_domain_bridge_4` ever moved our robot?~~ **Yes**, confirmed
   on 2026-11-13: the robot moves with `/cmd_vel` in auto mode.
2. **Does `lite3_domain_bridge_4` stop the robot when `/cmd_vel` goes quiet?** No timeout is
   documented for speed commands (§6.2). Test it in the open: send 0.1 m/s once, then stop
   publishing.
3. Why the robot became unsteady after the H-Step request on 2026-11-12.
4. Turn on depth-to-colour alignment in the Orbbec driver (allowed by the lab operator; see §3.1).
5. A way to read gait, mode and battery state. They're in the UDP state packet but the bridge
   doesn't publish them.
6. Current motion-host AI options (`0x21012109`: auto-stop / tracking). Is auto-stop on?
7. Real foot swing envelope and step height for planning.
8. One live check that `/leg_odom` `position.z` really carries yaw.
