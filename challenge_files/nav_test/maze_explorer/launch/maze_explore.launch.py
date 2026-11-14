"""
Maze exploration: /scan + slam_toolbox + Nav2 + cmd_vel relay + explore_lite + supervisor.

Every part can be switched off when something else already provides it, e.g. with the lab's
``lite3_launcher`` running scan/SLAM/Nav2/twist_mux::

    ros2 launch maze_explorer maze_explore.launch.py scan:=false slam:=false nav2:=false \
        relay:=false

Not started here: Fast-LIO + odom_relay (``odom -> base_link`` and ``/odom``) on the robot,
or the simulator (``sim.launch.py`` includes this file).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Build the launch description."""
    share = get_package_share_directory('maze_explorer')
    cfg = os.path.join(share, 'config')
    lc = LaunchConfiguration
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('scan', default_value='true',
                              description='pointcloud_to_laserscan: cloud -> /scan'),
        DeclareLaunchArgument('slam', default_value='true', description='slam_toolbox'),
        DeclareLaunchArgument('nav2', default_value='true', description='Nav2 servers'),
        DeclareLaunchArgument('relay', default_value='true',
                              description='cmd_vel_relay: /nav_cmd_vel -> /cmd_vel. Turn off '
                                          'if twist_mux already publishes /cmd_vel'),
        DeclareLaunchArgument('explore', default_value='true', description='explore_lite'),
        DeclareLaunchArgument('supervisor', default_value='true',
                              description='explore_supervisor (spins + camera coverage). '
                                          'false = plain explore_lite'),
        DeclareLaunchArgument('spin', default_value='true',
                              description='supervisor: 360 deg spin once per metre'),
        DeclareLaunchArgument('coverage', default_value='true',
                              description='supervisor: visit walls the camera has not seen'),
        DeclareLaunchArgument('debug', default_value='false',
                              description='explore_lite frontier markers'),
        DeclareLaunchArgument('gait', default_value='h_step',
                              description='supervisor: gait set before moving: none | h_step '
                                          '| rug_general | rug_grip (needs transfer_interfaces)'),
        DeclareLaunchArgument('gait_min_lift', default_value='0.0',
                              description='supervisor: foot lift (m) that confirms the gait; '
                                          '0 = only report it'),
        DeclareLaunchArgument('gait_check_fail', default_value='warn',
                              description='supervisor: warn | stop when the gait check fails'),
        DeclareLaunchArgument('entrance', default_value='',
                              description="supervisor: start-cell edges to close with a "
                                          "virtual wall, e.g. 'back' or 'back,left'"),
        DeclareLaunchArgument('fence_polygon', default_value='',
                              description="supervisor: 'x1,y1,x2,y2,...' in the start frame; "
                                          'everything outside is closed'),
        DeclareLaunchArgument('cloud_topic', default_value='/livox/points'),
        DeclareLaunchArgument('nav_cmd_vel', default_value='/nav_cmd_vel',
                              description='Nav2 velocity output (input of the relay)'),
        DeclareLaunchArgument('nav2_params', default_value=os.path.join(cfg, 'nav2_params.yaml')),
        DeclareLaunchArgument('slam_params', default_value=os.path.join(cfg, 'slam_toolbox.yaml')),
        DeclareLaunchArgument('scan_params', default_value=os.path.join(cfg, 'scan.yaml')),
        DeclareLaunchArgument('explore_params', default_value=os.path.join(cfg, 'explore.yaml')),
        DeclareLaunchArgument('supervisor_params',
                              default_value=os.path.join(cfg, 'supervisor.yaml')),
        DeclareLaunchArgument('relay_params', default_value=os.path.join(cfg, 'relay.yaml')),
    ]
    sim = {'use_sim_time': lc('use_sim_time')}
    # With the supervisor, explore_lite and the global costmap read its fenced map.
    map_topic = PythonExpression(["'/map_fenced' if '", lc('supervisor'),
                                  "'.lower() == 'true' else '/map'"])
    nav2_yaml = RewrittenYaml(source_file=lc('nav2_params'),
                              param_rewrites={'map_topic': map_topic}, convert_types=True)

    scan = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan', output='screen',
        parameters=[lc('scan_params'), sim],
        remappings=[('cloud_in', lc('cloud_topic')), ('scan', '/scan')],
        condition=IfCondition(lc('scan')))

    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
        output='screen', parameters=[lc('slam_params'), sim],
        condition=IfCondition(lc('slam')))

    nav_params = [nav2_yaml, sim]
    to_smoother = [('cmd_vel', 'cmd_vel_nav')]
    nav2 = GroupAction(condition=IfCondition(lc('nav2')), actions=[
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen',
             parameters=nav_params, remappings=to_smoother),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=nav_params),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=nav_params, remappings=to_smoother),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=nav_params),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=nav_params,
             remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', lc('nav_cmd_vel'))]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen', parameters=nav_params),
    ])

    relay = Node(
        package='maze_explorer', executable='cmd_vel_relay', name='cmd_vel_relay',
        output='screen',
        parameters=[lc('relay_params'), sim, {'input_topic': lc('nav_cmd_vel')}],
        condition=IfCondition(lc('relay')))

    explore = Node(
        package='explore_lite', executable='explore', name='explore_node', output='screen',
        parameters=[lc('explore_params'), sim,
                    {'visualize': lc('debug'), 'costmap_topic': map_topic}],
        condition=IfCondition(lc('explore')))

    supervisor = Node(
        package='maze_explorer', executable='explore_supervisor', name='explore_supervisor',
        output='screen',
        parameters=[lc('supervisor_params'), sim,
                    {'spin_enabled': lc('spin'), 'coverage_enabled': lc('coverage'),
                     'gait': lc('gait'), 'gait_min_lift_m': lc('gait_min_lift'),
                     'gait_check_fail': lc('gait_check_fail'),
                     'entrance': lc('entrance'), 'fence_polygon': lc('fence_polygon')}],
        condition=IfCondition(lc('supervisor')))

    return LaunchDescription(args + [scan, slam, nav2, relay, explore, supervisor])
