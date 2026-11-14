"""
Node ``explore_supervisor``: adds 360° spins and camera-coverage goals on top of explore_lite.

Phases (published on ``/maze_explorer/state``, latched):

0. ``gait`` (unless ``gait`` is ``none``): with explore_lite paused, send Move mode and then
   the gait (default RUG H-Step) on ``/simple_cmd``, once each, and wait for it to settle.
   The bridge doesn't report the gait, so it is *verified* from ``/joint_states`` during the
   first spin: the foot lift must reach ``gait_min_lift_m`` (see :mod:`gait_utils`). The
   result goes to ``/maze_explorer/gait_check`` (latched); ``gait_check_fail: stop`` stops
   the run if the check fails.
1. ``exploring``: explore_lite drives to frontiers. Whenever the robot is ``spin_spacing_m``
   from every earlier spin, the supervisor pauses explore_lite (``/explore/resume`` false),
   runs Nav2's Spin behaviour once round, and resumes it. This covers the lidar's blind spots
   (it can't see the low maze walls behind or close beside the robot) and turns the camera
   to every wall.
2. ``coverage``: when explore_lite reports ``exploration_complete``, the supervisor drives to
   poses from which the camera sees wall faces it hasn't seen yet (dead ends the lidar mapped
   from a distance), nearest first, and turns to face them.
3. If coverage found anything, explore_lite is resumed once more in case the map grew
   (``max_rounds``), then ``done``.

The camera footprint is marked on the map all the time (every ``mark_period`` s) from the TF
of ``camera_frame``.

Virtual fence: the supervisor republishes ``/map`` as ``/map_fenced`` (latched) with virtual
walls added: across the start cell's ``entrance`` edges and/or outside ``fence_polygon``,
both given in the *start frame* (the robot's pose when the supervisor first gets TF: origin
at the start cell's centre, x ahead). explore_lite and Nav2's global costmap read
``/map_fenced``, so no frontier or path leads out of the maze. ``/map_fenced`` is only
published once the start pose is known, which also holds explore_lite back until then.

The pure logic is in :mod:`maze_explorer.coverage_utils`.
"""

import math

from action_msgs.msg import GoalStatus
import cv2
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
import tf2_ros

from maze_explorer.coverage_utils import (angle_diff, apply_fence, CoverageMap,
                                          entrance_segments, GridInfo, parse_edges,
                                          parse_polygon, plan_view_goal, reachable_area,
                                          SpinPlanner, start_to_world, yaw_from_quaternion)
from maze_explorer.gait_utils import StepHeightMeter

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
EXPLORE_COMPLETE = 'exploration_complete'
# /simple_cmd codes (robot_info.md §6.3). Gait changes need Move mode first.
MOVE_MODE = 0x21010D06
GAITS = {'none': None, 'h_step': 0x21010407, 'rug_general': 0x21010401,
         'rug_grip': 0x21010402}


class ExploreSupervisor(Node):
    """State machine around explore_lite; see the module docstring."""

    def __init__(self):
        """Declare parameters, connect topics/actions and start the timer."""
        super().__init__('explore_supervisor')
        p = self.declare_parameter
        self.spin_enabled = p('spin_enabled', True).value
        self.spin_spacing = p('spin_spacing_m', 1.0).value
        self.spin_angle = p('spin_angle', 2.0 * math.pi).value
        self.spin_time = p('spin_time_allowance', 40.0).value
        self.coverage_enabled = p('coverage_enabled', True).value
        self.camera_frame = p('camera_frame', 'camera_link').value
        self.hfov = math.radians(p('camera_hfov_deg', 70.0).value)
        self.cam_range = p('camera_max_range', 2.5).value
        self.cam_min_range = p('camera_min_range', 0.3).value
        self.max_incidence = math.radians(p('max_incidence_deg', 60.0).value)
        self.mark_period = p('mark_period', 0.25).value
        self.robot_radius = p('robot_radius', 0.25).value
        self.view_min = p('view_min', 0.6).value
        self.view_ideal = p('view_ideal', 1.0).value
        self.view_max = p('view_max', 1.6).value
        self.min_cluster_m = p('min_cluster_m', 0.3).value
        self.goal_timeout = p('coverage_goal_timeout', 90.0).value
        self.dwell = p('dwell_s', 1.0).value
        self.max_rounds = p('max_rounds', 3).value
        self.global_frame = p('global_frame', 'map').value
        self.base_frame = p('base_frame', 'base_link').value
        map_topic = p('map_topic', '/map').value
        fenced_topic = p('fenced_map_topic', '/map_fenced').value
        self.entrance = parse_edges(p('entrance', '').value)
        self.fence_polygon = parse_polygon(p('fence_polygon', '').value)
        self.cell_size = p('cell_size', 1.0).value
        self.fence_thickness = p('fence_thickness', 0.1).value
        self.fence_seal = p('fence_seal_m', 0.5).value
        self.stall_timeout = p('stall_timeout_s', 90.0).value
        self.stall_distance = p('stall_distance_m', 0.25).value
        gait = p('gait', 'h_step').value.strip().lower()
        if gait not in GAITS:
            raise ValueError(f'gait must be one of {sorted(GAITS)}, not {gait!r}')
        self.gait = gait
        self.gait_topic = p('gait_topic', '/simple_cmd').value
        self.gait_wait = p('gait_wait_s', 10.0).value
        self.gait_settle = p('gait_settle_s', 2.0).value
        self.gait_min_lift = p('gait_min_lift_m', 0.0).value
        self.gait_fail_action = p('gait_check_fail', 'warn').value.strip().lower()
        if self.gait_fail_action not in ('warn', 'stop'):
            raise ValueError("gait_check_fail must be 'warn' or 'stop'")
        joint_topic = p('joint_states_topic', '/joint_states').value
        status_topic = p('explore_status_topic', '/explore/status').value
        resume_topic = p('explore_resume_topic', '/explore/resume').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(OccupancyGrid, map_topic, self.on_map, LATCHED)
        self.status_sub = None
        self.status_topic = status_topic
        self.subscribe_status()
        self.resume_pub = self.create_publisher(Bool, resume_topic, 10)
        self.state_pub = self.create_publisher(String, '/maze_explorer/state', LATCHED)
        self.coverage_pub = self.create_publisher(OccupancyGrid, '/maze_explorer/coverage', 1)
        self.fenced_pub = self.create_publisher(OccupancyGrid, fenced_topic, LATCHED)
        self.gait_pub = None
        self.gait_check_pub = self.create_publisher(String, '/maze_explorer/gait_check', LATCHED)
        self.meter = StepHeightMeter(window_s=8.0)
        self.gait_check_pending = False
        if self.gait != 'none':
            self.gait_pub = self.make_gait_publisher()
            self.create_subscription(JointState, joint_topic, self.on_joints, 50)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')

        self.grid = None
        self.info = None
        self.map_header = None
        self.raw_map = None
        self.start = None
        self.fence_segments = []
        self.fence_world_polygon = None
        self.gait_phase = 0
        self.gait_phase_since = 0.0
        self.stall_anchor = None  # (x, y, exploring time) where the robot last made progress
        self.explore_time = 0.0   # seconds spent in 'exploring' (spins etc. don't count)
        self.last_tick = None
        self.coverage = CoverageMap()
        self.spins = SpinPlanner(self.spin_spacing)
        self.explore_complete = False
        self.explore_running = False
        self.warned_explore = False
        self.state = None
        self.state_since = self.now_s()
        self.goal_handle = None
        self.goal_token = 0
        self.view_goal = None
        self.blacklist = []
        self.rounds = 1
        self.round_goals = 0
        self.after_spin = 'exploring'
        self.set_state('waiting')
        self.last_mark = 0.0
        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f'spin {"on" if self.spin_enabled else "off"} (every {self.spin_spacing} m), '
            f'coverage {"on" if self.coverage_enabled else "off"}, gait {self.gait}, '
            f'entrance walls {self.entrance or "none"}, '
            f'fence polygon {"set" if self.fence_polygon else "none"}')

    # ---------------------------------------------------------------- helpers
    def subscribe_status(self):
        """Subscribe to explore_lite's latched status (type comes from explore_lite_msgs)."""
        try:
            from explore_lite_msgs.msg import ExploreStatus
        except ImportError:
            self.get_logger().error('explore_lite_msgs not found: build m-explore-ros2 first')
            raise
        self.status_sub = self.create_subscription(
            ExploreStatus, self.status_topic, self.on_status, LATCHED)

    def make_gait_publisher(self):
        """Create the ``/simple_cmd`` publisher with the bridge's QoS (robot_info.md §6.3)."""
        try:
            from transfer_interfaces.msg import MotionSimpleCMD
        except ImportError:
            self.get_logger().error(
                'transfer_interfaces not found: source ~/lite3_ws (robot) or set gait:=none')
            raise
        self.simple_cmd_type = MotionSimpleCMD
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         lifespan=Duration(seconds=1.0))
        return self.create_publisher(MotionSimpleCMD, self.gait_topic, qos)

    def send_simple_cmd(self, code):
        """Send one ``/simple_cmd`` command (sent once; these aren't streams)."""
        self.gait_pub.publish(self.simple_cmd_type(cmd_code=code, size=0, type=0))
        self.get_logger().info(f'sent /simple_cmd 0x{code:08X}')

    def now_s(self):
        """Return the node clock in seconds (sim time when use_sim_time is set)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, state):
        """Switch state, publish it and log it."""
        if state != self.state:
            self.last_tick = None  # the stall timer only runs while exploring
            self.get_logger().info(f'state: {self.state} -> {state}')
            self.state = state
            self.state_since = self.now_s()
            self.state_pub.publish(String(data=state))

    def lookup(self, frame):
        """Return (x, y, yaw) of ``frame`` in the global frame, or None if TF isn't ready."""
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, frame, Time())
        except tf2_ros.TransformException:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def resume_explore(self, on):
        """Pause or resume explore_lite."""
        self.resume_pub.publish(Bool(data=on))

    # ---------------------------------------------------------------- callbacks
    def on_map(self, msg):
        """Keep the latest map; once the start pose is known, fence it and republish it."""
        self.raw_map = msg
        if self.start is not None:
            self.process_map(msg)

    def set_start(self, robot):
        """Fix the start frame at the robot's pose and turn the fence into map coordinates."""
        self.start = robot
        segs = entrance_segments(self.entrance, self.cell_size)
        self.fence_segments = []
        for x0, y0, x1, y1 in segs:
            (a, b), (c, d) = start_to_world(robot, [(x0, y0), (x1, y1)])
            self.fence_segments.append((a, b, c, d))
        if self.fence_polygon:
            self.fence_world_polygon = start_to_world(robot, self.fence_polygon)
        self.get_logger().info(f'start pose ({robot[0]:.2f}, {robot[1]:.2f}, '
                               f'{math.degrees(robot[2]):.0f} deg); '
                               f'{len(self.fence_segments)} virtual wall(s)')
        if self.raw_map is not None:
            self.process_map(self.raw_map)

    def process_map(self, msg):
        """Apply the virtual fence, publish ``/map_fenced`` and follow the map's growth."""
        o = msg.info.origin.position
        self.info = GridInfo(msg.info.resolution, o.x, o.y, msg.info.width, msg.info.height)
        raw = np.asarray(msg.data, np.int8).reshape(msg.info.height, msg.info.width)
        grid, virtual = apply_fence(raw, self.info, self.fence_segments,
                                    self.fence_world_polygon, self.fence_thickness,
                                    self.fence_seal)
        self.grid = grid
        self.map_header = msg.header
        self.coverage.update_grid(self.info)
        if virtual.any():
            # Nothing to read on a virtual wall: count the floor in front of it as seen.
            near = cv2.dilate(virtual.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
            self.coverage.seen |= near & ~virtual
        out = OccupancyGrid()
        out.header, out.info = msg.header, msg.info
        out.data = grid.ravel().tolist()
        self.fenced_pub.publish(out)

    def on_joints(self, msg):
        """Feed joint angles to the step-height meter (receipt time; stamps are steady clock)."""
        self.meter.add(self.now_s(), msg.name, msg.position)

    def check_gait(self):
        """
        Verify the gait change from the foot lift measured during the first spin.

        Only meaningful while the robot was stepping (it spins in place right after the gait
        change). With ``gait_min_lift_m`` 0 (not calibrated) the lift is only reported.
        """
        self.gait_check_pending = False
        lift = self.meter.lift()
        if lift is None or self.meter.span_s() < 3.0:
            result = f'{self.gait} UNVERIFIED: no /joint_states during the first spin'
            ok = self.gait_min_lift <= 0.0
        else:
            legs = ', '.join(f'{k} {v * 100:.1f}' for k, v in sorted(self.meter.lifts().items()))
            if self.gait_min_lift <= 0.0:
                result = (f'{self.gait} UNVERIFIED (gait_min_lift_m not calibrated): foot lift '
                          f'{lift * 100:.1f} cm [{legs}]')
                ok = True
            else:
                ok = lift >= self.gait_min_lift
                result = (f'{self.gait} {"OK" if ok else "FAILED"}: foot lift {lift * 100:.1f} '
                          f'cm, need >= {self.gait_min_lift * 100:.1f} cm [{legs}]')
        log = self.get_logger().info if ok else self.get_logger().error
        log(f'gait check: {result}')
        self.gait_check_pub.publish(String(data=result))
        if not ok and self.gait_fail_action == 'stop':
            self.get_logger().error('gait check failed and gait_check_fail is stop: stopping')
            self.resume_explore(False)
            self.cancel_nav()
            self.set_state('done')
            return True
        return False

    def on_status(self, msg):
        """Track explore_lite's status: running at all, and ``exploration_complete``."""
        self.explore_running = True
        if msg.status == EXPLORE_COMPLETE:
            self.explore_complete = True

    # ---------------------------------------------------------------- main loop
    def tick(self):
        """Mark camera coverage and step the state machine."""
        now = self.now_s()
        if self.start is None:
            robot = self.lookup(self.base_frame)
            if robot is None:
                return
            self.set_start(robot)
        if self.grid is not None and now - self.last_mark >= self.mark_period:
            self.last_mark = now
            cam = self.lookup(self.camera_frame)
            if cam is not None:
                self.coverage.mark_view(self.grid, cam[0], cam[1], cam[2], self.hfov,
                                        self.cam_range, self.cam_min_range,
                                        self.max_incidence)
                self.publish_coverage()
        robot = self.lookup(self.base_frame)
        getattr(self, 'step_' + self.state)(robot, now)

    def step_waiting(self, robot, now):
        """
        Wait for the map, TF, the Nav2 action servers and explore_lite.

        explore_lite publishes its first status only once it is connected to Nav2, and a
        pause sent before it subscribes to ``/explore/resume`` would be lost.
        """
        if robot is None or self.grid is None:
            return
        if not self.nav.server_is_ready() or not self.spin_client.server_is_ready():
            return
        if not self.explore_running or self.resume_pub.get_subscription_count() == 0:
            if now - self.state_since > 30.0 and not self.warned_explore:
                self.warned_explore = True
                self.get_logger().warn('still waiting for explore_lite (/explore/status)')
            return
        if self.gait_pub is not None:
            self.resume_explore(False)
            self.gait_phase = 0
            self.set_state('gait')
        else:
            self.set_state('exploring')

    def step_gait(self, robot, now):
        """
        Set the gait before moving: Move mode, then the gait code, each sent once.

        explore_lite is paused meanwhile. The bridge creates its ``/simple_cmd`` subscription
        only after it sees a publisher, so wait (up to ``gait_wait_s``) for it to match.
        The bridge reports no gait state, so success can't be confirmed from ROS.
        """
        t = now - self.state_since
        if self.gait_phase == 0:
            matched = self.gait_pub.get_subscription_count() > 0
            if t < 1.0 or (not matched and t < self.gait_wait):
                return  # let the robot stop and the bridge subscribe
            if not matched:
                self.get_logger().error(f'no subscriber on {self.gait_topic} after '
                                        f'{self.gait_wait:.0f} s; gait NOT changed')
                self.gait_check_pub.publish(String(
                    data=f'{self.gait} NOT SENT: no subscriber on {self.gait_topic}'))
                self.end_gait()
                return
            self.send_simple_cmd(MOVE_MODE)
            self.gait_phase, self.gait_phase_since = 1, now
        elif self.gait_phase == 1 and now - self.gait_phase_since >= 1.0:
            self.send_simple_cmd(GAITS[self.gait])
            self.gait_phase, self.gait_phase_since = 2, now
        elif self.gait_phase == 2 and now - self.gait_phase_since >= self.gait_settle:
            self.get_logger().info(f'gait {self.gait} requested; it is checked from '
                                   f'/joint_states during the first spin')
            self.gait_check_pending = True
            self.end_gait()

    def end_gait(self):
        """Leave the gait phase and let explore_lite drive."""
        self.resume_explore(True)
        self.set_state('exploring')

    def stalled(self, robot, now):
        """
        Return True when explore_lite has made no progress for ``stall_timeout_s``.

        Counts only time spent in ``exploring``. explore_lite blacklists frontiers it can't
        reach, but not ones Nav2 reports as reached without the map changing (it then sends
        the same goal forever); this watchdog ends such a loop.
        """
        if self.last_tick is not None:
            self.explore_time += min(now - self.last_tick, 1.0)
        self.last_tick = now
        if robot is None:
            return False
        if (self.stall_anchor is None or math.hypot(robot[0] - self.stall_anchor[0],
                                                    robot[1] - self.stall_anchor[1])
                >= self.stall_distance):
            self.stall_anchor = (robot[0], robot[1], self.explore_time)
            return False
        return self.explore_time - self.stall_anchor[2] > self.stall_timeout

    def step_exploring(self, robot, now):
        """Let explore_lite run; interrupt it for spins; move on when it's complete."""
        if not self.explore_complete and self.stalled(robot, now):
            self.get_logger().warn(f'no progress for {self.stall_timeout:.0f} s while '
                                   f'exploring; treating exploration as finished')
            self.resume_explore(False)
            self.explore_complete = True
            self.stall_anchor = None
        if (self.gait_check_pending and not self.spin_enabled
                and now - self.state_since > 15.0 and self.meter.span_s() >= 6.0):
            if self.check_gait():  # no spins: judge the gait from the first 15 s of driving
                return
        if self.explore_complete:
            self.get_logger().info(f'explore_lite finished round {self.rounds}')
            if self.spin_enabled and self.spins.should_spin(robot[0], robot[1]):
                self.start_spin(robot, 'after_explore')
            else:
                self.after_explore()
            return
        if self.spin_enabled and robot is not None and self.spins.should_spin(robot[0],
                                                                              robot[1]):
            self.resume_explore(False)
            self.after_spin = 'exploring'
            self.set_state('pausing')

    def step_pausing(self, robot, now):
        """Give explore_lite/Nav2 half a second to cancel the frontier goal, then spin."""
        if now - self.state_since >= 0.5:
            self.start_spin(robot, self.after_spin)

    def step_spinning(self, robot, now):
        """Wait for the Spin action (its result callback moves on)."""

    def step_after_explore(self, robot, now):
        """Spin after explore_lite finished is done; go on."""
        self.after_explore()

    def after_explore(self):
        """Explore round finished: start coverage or finish."""
        if self.coverage_enabled:
            self.round_goals = 0
            self.set_state('coverage')
        else:
            self.finish()

    def step_coverage(self, robot, now):
        """Pick the next view goal, or resume exploring / finish when none is left."""
        if robot is None or self.grid is None:
            return
        cells = max(1, int(round(self.min_cluster_m / self.info.resolution)))
        goal = plan_view_goal(self.grid, self.info, self.coverage, robot[:2],
                              self.robot_radius, self.view_min, self.view_ideal,
                              self.view_max, cells, self.blacklist)
        if goal is None:
            if self.round_goals > 0 and self.rounds < self.max_rounds:
                self.rounds += 1
                self.explore_complete = False
                self.get_logger().info(f'coverage round done, exploring again '
                                       f'(round {self.rounds}/{self.max_rounds})')
                self.resume_explore(True)
                self.set_state('exploring')
            else:
                self.finish()
            return
        self.view_goal = goal
        self.round_goals += 1
        self.get_logger().info(
            f'coverage goal ({goal.x:.2f}, {goal.y:.2f}) looking at ({goal.aim_x:.2f}, '
            f'{goal.aim_y:.2f}), {goal.cluster_cells} unseen cells, {goal.travel_m:.1f} m away')
        self.send_nav(goal)
        self.set_state('coverage_nav')

    def step_coverage_nav(self, robot, now):
        """Drive to the view goal; cancel early once the aim point has been seen."""
        g = self.view_goal
        r, c = self.info.world_to_cell(g.aim_x, g.aim_y)
        seen = self.info.inside(r, c) and self.coverage.seen[r, c]
        if seen or now - self.state_since > self.goal_timeout:
            if not seen:
                self.get_logger().warn('coverage goal timed out; skipping it')
                self.blacklist.append((g.aim_x, g.aim_y))
            self.cancel_nav()
            self.set_state('coverage')

    def step_turning(self, robot, now):
        """Wait for the turn towards the aim point (Spin result callback moves on)."""

    def step_dwell(self, robot, now):
        """Stand still for ``dwell_s`` so perception gets a few sharp frames."""
        if now - self.state_since >= self.dwell:
            g = self.view_goal
            r, c = self.info.world_to_cell(g.aim_x, g.aim_y)
            if not (self.info.inside(r, c) and self.coverage.seen[r, c]):
                self.blacklist.append((g.aim_x, g.aim_y))  # looked, still unseen: skip it
            self.set_state('coverage')

    def step_done(self, robot, now):
        """Nothing left to do."""

    def finish(self):
        """Log the coverage summary and stop."""
        robot = self.lookup(self.base_frame)
        if self.grid is not None and robot is not None:
            _, swept = reachable_area(self.grid, self.info, robot[:2], self.robot_radius)
            seen, total = self.coverage.stats(self.grid, swept)
            pct = 100.0 * seen / total if total else 100.0
            area = np.count_nonzero(swept) * self.info.resolution ** 2
            self.get_logger().info(f'done: {area:.1f} m2 reachable, camera saw {seen}/{total} '
                                   f'reachable wall-face cells ({pct:.0f} %), '
                                   f'{len(self.spins.points)} spins, '
                                   f'{len(self.blacklist)} views skipped')
        self.set_state('done')

    # ---------------------------------------------------------------- actions
    def start_spin(self, robot, next_state):
        """Spin once round with Nav2's Spin behaviour, then go to ``next_state``."""
        if robot is not None:
            self.spins.record(robot[0], robot[1])
        self.send_spin(self.spin_angle, next_state, resume_after=(next_state == 'exploring'))
        self.set_state('spinning')

    def send_spin(self, angle, next_state, resume_after=False):
        """Send a Spin goal of ``angle`` rad; on any result go to ``next_state``."""
        goal = Spin.Goal()
        goal.target_yaw = float(angle)
        goal.time_allowance = Duration(seconds=self.spin_time).to_msg()
        self.goal_token += 1
        token = self.goal_token

        def done(fut):
            if token != self.goal_token:
                return
            status = fut.result().status if fut.result() is not None else None
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().warn(f'spin ended with status {status}')
            # Only a full spin is a fair test: an aborted one mostly measures standing still.
            if (self.gait_check_pending and status == GoalStatus.STATUS_SUCCEEDED
                    and self.check_gait()):
                return
            if resume_after:
                self.resume_explore(True)
            self.set_state(next_state)

        def accepted(fut):
            handle = fut.result()
            if handle is None or not handle.accepted:
                self.get_logger().warn('spin goal rejected')
                if token == self.goal_token:
                    if resume_after:
                        self.resume_explore(True)
                    self.set_state(next_state)
                return
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(done)

        self.spin_client.send_goal_async(goal).add_done_callback(accepted)

    def send_nav(self, g):
        """Send a NavigateToPose goal to view goal ``g``."""
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = g.x, g.y
        pose.pose.orientation.z = math.sin(g.yaw / 2.0)
        pose.pose.orientation.w = math.cos(g.yaw / 2.0)
        goal.pose = pose
        self.goal_token += 1
        token = self.goal_token

        def done(fut):
            if token != self.goal_token or self.state != 'coverage_nav':
                return
            status = fut.result().status if fut.result() is not None else None
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.turn_to_aim()
            else:
                self.get_logger().warn(f'coverage goal failed (status {status}); skipping it')
                self.blacklist.append((g.aim_x, g.aim_y))
                self.set_state('coverage')

        def accepted(fut):
            handle = fut.result()
            if handle is None or not handle.accepted:
                if token == self.goal_token and self.state == 'coverage_nav':
                    self.get_logger().warn('coverage goal rejected; skipping it')
                    self.blacklist.append((g.aim_x, g.aim_y))
                    self.set_state('coverage')
                return
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(done)

        self.nav.send_goal_async(goal).add_done_callback(accepted)

    def turn_to_aim(self):
        """At the view goal: turn the camera to the aim point (Nav2 ignores goal yaw)."""
        robot = self.lookup(self.base_frame)
        g = self.view_goal
        if robot is None:
            self.set_state('dwell')
            return
        diff = angle_diff(math.atan2(g.aim_y - robot[1], g.aim_x - robot[0]), robot[2])
        if abs(diff) < 0.2:
            self.set_state('dwell')
            return
        self.send_spin(diff, 'dwell')
        self.set_state('turning')

    def cancel_nav(self):
        """Cancel the current goal (if any) and ignore its late result."""
        self.goal_token += 1
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    # ---------------------------------------------------------------- debug output
    def publish_coverage(self):
        """Publish wall faces (100 unseen, 0 seen) for RViz, only while someone listens."""
        if self.coverage_pub.get_subscription_count() == 0:
            return
        msg = OccupancyGrid()
        msg.header = self.map_header
        i = self.info
        msg.info.resolution = i.resolution
        msg.info.width, msg.info.height = i.width, i.height
        msg.info.origin.position.x, msg.info.origin.position.y = i.origin_x, i.origin_y
        msg.info.origin.orientation.w = 1.0
        data = np.full((i.height, i.width), -1, np.int8)
        faces = self.coverage.unseen_faces(self.grid)
        data[self.coverage.seen] = 0
        data[faces] = 100
        msg.data = data.ravel().tolist()
        self.coverage_pub.publish(msg)

    def shutdown(self):
        """Cancel our goals so nothing keeps driving after we exit."""
        self.cancel_nav()


def main(args=None):
    """Run the supervisor until Ctrl+C."""
    rclpy.init(args=args)
    node = ExploreSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
