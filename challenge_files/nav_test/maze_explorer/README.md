# maze_explorer

Autonomous exploration of a maze of 1 m cells with ~45 cm walls, for the DEEP Robotics Lite3.
It uses **slam_toolbox** (map), **Nav2** (driving) and **explore_lite** (frontier
exploration), plus a small supervisor that makes sure the camera actually looks at every
wall, so QR codes and posters in dead ends aren't missed.

```
 /livox/points ─► pointcloud_to_laserscan ─► /scan ─► slam_toolbox ─► /map, map→odom
                   (band for 45 cm walls)       │
                                                ▼
 explore_lite ── frontier goals ──► Nav2 (planner, RPP controller, Spin, smoother)
      ▲  │ /explore/status                          │ /nav_cmd_vel
      │  ▼                                          ▼
 explore_supervisor ── spins, coverage goals ─►  cmd_vel_relay ─► /cmd_vel ─► robot bridge
   (camera coverage of the walls)                (bridge QoS, clamping, stop on silence)
```

Robot interface details (topics, TF, QoS, safety rules) are in `../../robot_info.md`.

## What it does

1. **Explore** (explore_lite): drive to the nearest edge between known and unknown space
   until the map has none left.
2. **Spin** (supervisor): every time the robot reaches ground ≥ 1 m from any earlier spin, pause
   explore_lite and turn 360° on the spot. The lidar sits only ~2.5 cm above the wall tops,
   so it sees walls well in front but barely beside and never behind the robot; the spin
   fills those gaps and swings the camera past every wall.
3. **Coverage** (supervisor): the camera's view is traced on the map all the time. When
   exploration ends, the supervisor drives to poses facing wall pieces the camera hasn't
   seen yet (nearest first), turns to them and waits 1 s. Walls seen only from far away or at
   a grazing angle (> 60°) don't count.
4. If coverage visited anything, explore_lite runs once more (the map may have grown), then
   the state goes to `done` and a summary is logged.

Each part can be switched off; `supervisor:=false` gives plain explore_lite.

## Files

| Path | What |
|---|---|
| `launch/maze_explore.launch.py` | The stack, every part switchable (robot and sim) |
| `launch/sim.launch.py` | Gazebo test: maze world + Lite3 stand-in + the stack |
| `maze_explorer/explore_supervisor_node.py` | Node `explore_supervisor` |
| `maze_explorer/cmd_vel_relay_node.py` | Node `cmd_vel_relay` |
| `maze_explorer/coverage_utils.py` | ROS-free coverage/viewpoint logic (unit-tested) |
| `maze_explorer/maze_world.py` | ROS-free maze text → Gazebo world (unit-tested) |
| `config/*.yaml` | Parameters: `nav2_params`, `slam_toolbox`, `scan`, `explore`, `supervisor`, `relay` |
| `sim/lite3_sim.sdf`, `sim/mazes/*.txt` | Simulated robot; `example_maze` (layout of `example maze.step`), `test_maze` (5 × 4 with a loop) |
| `rviz/maze_explore.rviz` | RViz view: map, coverage, scan, frontiers, plan, footprint |
| `install_deps.sh` | apt packages + clones explore_lite |

## Install and build

explore_lite has no Humble apt package, so it is built from source
([m-explore-ros2](https://github.com/robo-friends/m-explore-ros2), pinned commit).

1. Copy this `maze_explorer` folder into a workspace's `src/` (e.g. `~/challenge_ws/src/`;
   not inside `robot_challenge_workspace/src/challenge_files`, which colcon ignores).
2. Install the dependencies and clone explore_lite next to the package:
   ```bash
   bash ~/challenge_ws/src/maze_explorer/install_deps.sh            # dev PC (with Gazebo)
   bash ~/challenge_ws/src/maze_explorer/install_deps.sh --no-sim   # robot computer
   ```
   `sudo` needs a password; use `--no-apt` if the apt packages are already there.
3. Build (from the workspace root, never from `src/`):
   ```bash
   cd ~/challenge_ws
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install --packages-select explore_lite_msgs explore_lite maze_explorer
   source install/setup.bash
   ```
   explore_lite takes about a minute to compile.
4. Tests (optional): `cd src/maze_explorer && python3 -m pytest test -q` (31 tests,
   including flake8 and pep257).

## Try it in simulation first

Use a domain the robot isn't on, so simulated commands can't reach it:

```bash
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1
ros2 launch maze_explorer sim.launch.py                         # example maze, headless
ros2 launch maze_explorer sim.launch.py maze:=test_maze rviz:=true
ros2 launch maze_explorer sim.launch.py gui:=true               # with the Gazebo window
ros2 launch maze_explorer sim.launch.py supervisor:=false       # plain explore_lite
```

- Gazebo takes 20–50 s to start; the robot then spins at the start cell and sets off.
- Watch the phase: `ros2 topic echo /maze_explorer/state` (`waiting`, `exploring`, `pausing`,
  `spinning`, `coverage`, `coverage_nav`, `turning`, `dwell`, `done`).
- The end of the log has the summary, e.g. from the test maze:
  `done: 16.9 m2 reachable, camera saw 667/745 reachable wall-face cells (90 %), 8 spins`.
- **Your own maze:** write a text file like `sim/mazes/test_maze.txt` and pass its path,
  `maze:=/path/my_maze.txt`. Format: `+` corners, `---` / `|` walls, spaces for openings;
  cell contents `   ` (normal), ` S ` (start, exactly one), `###` (solid block, e.g. stairs),
  `...` (not part of the maze, walled off automatically). Full description at the top of
  `maze_explorer/maze_world.py`.
- The simulated robot is a box on a holonomic base, with a lidar at the real mount and the
  Mid-360's −7…+52° vertical field of view (2° steps, coarser than the real scan pattern).
  It has no legs, no camera image and perfect odometry, so on the robot expect slower,
  less tidy runs.

## Run on the robot

**Before every run:** read the safety rules in `robot_info.md` §6. Someone holds the
handheld controller with STOP next to the robot, the operator stands it up and switches it to
auto mode (`lite3_auto`), and only one node publishes `/cmd_vel`
(`ros2 topic info -v /cmd_vel`).

Prerequisite: **Fast-LIO + `odom_relay`** must be running (they provide `odom → base_link`
and `/odom`; this package doesn't start them). Check: `ros2 run tf2_ros tf2_echo odom base_link`.

### A. Our whole stack (lab SLAM/Nav2 not running)

```bash
export ROS_DOMAIN_ID=4
export FASTRTPS_DEFAULT_PROFILES_FILE=~/challenge_ws/src/archive/config/fastdds_udp.xml  # see robot_info §8
ros2 launch maze_explorer maze_explore.launch.py
```

### B. On top of the lab's `lite3_launcher` (it already runs scan, SLAM, Nav2, twist_mux)

```bash
ros2 launch maze_explorer maze_explore.launch.py scan:=false slam:=false nav2:=false relay:=false
```

The lab Nav2 must have the `spin` behaviour and publish `/navigate_to_pose`; its `/scan`
height band (0.1–0.5 m above `base_link`) is mostly above the maze walls, so mode A works
better in the maze.

### Launch arguments (`maze_explore.launch.py`)

| Argument | Default | Meaning |
|---|---|---|
| `scan`, `slam`, `nav2`, `relay`, `explore` | `true` | Start pointcloud_to_laserscan / slam_toolbox / Nav2 / cmd_vel_relay / explore_lite |
| `supervisor` | `true` | Start `explore_supervisor`. `false` = plain explore_lite |
| `spin` | `true` | Supervisor: 360° spin once per metre of new ground |
| `coverage` | `true` | Supervisor: visit walls the camera hasn't seen after exploring |
| `debug` | `false` | explore_lite publishes frontier markers (`/explore/frontiers`) |
| `use_sim_time` | `false` | Set by `sim.launch.py` |
| `cloud_topic` | `/livox/points` | Lidar cloud for `/scan` |
| `nav_cmd_vel` | `/nav_cmd_vel` | Nav2's velocity output (input of the relay or the lab twist_mux) |
| `nav2_params`, `slam_params`, `scan_params`, `explore_params`, `supervisor_params`, `relay_params` | `config/*.yaml` | Use your own parameter files |

`sim.launch.py` passes all of these on and adds `maze`, `gui` and `rviz`.

### Stopping

Ctrl+C on the launch. The relay sends a zero Twist on shutdown, and one when Nav2 goes quiet.
Then have the operator send `lite3_manual`. For an emergency: the handheld STOP.

### Watching from a laptop

On the lab network with `ROS_DOMAIN_ID=4`:
`rviz2 -d $(ros2 pkg prefix maze_explorer)/share/maze_explorer/rviz/maze_explore.rviz`.
The coverage layer (`/maze_explorer/coverage`) is only computed while something subscribes
to it: red = wall the camera hasn't seen, green = seen.

## Topics

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/livox/points` | `PointCloud2` | in | Lidar |
| `/scan` | `LaserScan` | internal | Wall band of the cloud (−0.15…+0.08 m in `base_link`) |
| `/map` | `OccupancyGrid` | internal/out | slam_toolbox, latched; also used by `team_perception`'s `poster_detector` |
| `/explore/status`, `/explore/resume` | explore_lite_msgs `ExploreStatus`, `Bool` | internal | Supervisor ↔ explore_lite |
| `/navigate_to_pose`, `/spin` | Nav2 actions | internal | Used by explore_lite and the supervisor |
| `/nav_cmd_vel` | `Twist` | internal | Nav2 output |
| `/cmd_vel` | `Twist` | out | RELIABLE + TRANSIENT_LOCAL, lifespan 0.3 s, clamped to 0.25 m/s and 0.5 rad/s |
| `/maze_explorer/state` | `String` (latched) | out | Supervisor phase; `done` when finished |
| `/maze_explorer/coverage` | `OccupancyGrid` | out (debug) | Only while subscribed |

## Tuning

| Symptom | Parameter (file) |
|---|---|
| Walls missing from `/map`, or floor/terrain showing up as walls | `min_height` / `max_height` (`scan.yaml`). The band must cover the walls (top ≈ +0.10 m in `base_link`) and stay above terrain cells |
| Robot's own legs appear as obstacles | `range_min` (`scan.yaml`) |
| "Spin" fails (status 6) in cells | Footprint (`nav2_params.yaml`, 0.60 × 0.42 m now). It must turn inside a 0.95 m cell |
| Doorways look closed / no path | `inflation_radius` (0.40), `footprint` (`nav2_params.yaml`) |
| Spins take too long | `spin_spacing_m` (1.0) or `spin:=false` |
| Coverage goals for walls already readable | `camera_max_range` (2.5 m), `max_incidence_deg` (60), `min_cluster_m` (0.3) (`supervisor.yaml`) |
| explore_lite gives up on frontiers too soon or too late | `progress_timeout` (30 s), `min_frontier_size` (0.5 m) (`explore.yaml`) |
| Speeds | `desired_linear_vel` (RPP), `max_velocity` (smoother), `max_rotational_vel` (behaviours), relay limits; the team limits are 0.25 m/s and 0.5 rad/s |

## Known limits

- **The maze must be closed.** explore_lite follows every frontier, so an opening to the lab
  sends the robot out of the maze. Close openings, or watch and stop it.
- **Stairs and raised terrain:** the scan band ignores anything up to ~0.15 m (so terrain
  cells stay drivable), and the stairs block is an obstacle. Climbing needs a gait change
  that only the operator should request (`robot_info.md` §6.3).
- **Camera model:** coverage uses a flat 70° wedge out to 2.5 m from `camera_link`. It doesn't
  check image quality, and the lidar map limits what counts as a wall.
- **Low walls:** the lidar sees partly over the walls, so `/map` can show free space inside
  closed-off areas. Coverage ignores space the robot can't reach, but explore_lite may chase
  such frontiers until its `progress_timeout` blacklists them.
- **Not yet run on the real robot.** Only the simulation (example and test maze) has been
  tested. Expect to tune `scan.yaml` and the footprint first.
