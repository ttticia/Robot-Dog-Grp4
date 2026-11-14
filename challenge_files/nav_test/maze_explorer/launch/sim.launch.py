"""
Gazebo test of the maze exploration: maze world + Lite3 stand-in + maze_explore.launch.py.

    ros2 launch maze_explorer sim.launch.py                       # example maze, headless
    ros2 launch maze_explorer sim.launch.py maze:=test_maze gui:=true
    ros2 launch maze_explorer sim.launch.py maze:=/path/to/my_maze.txt supervisor:=false

``maze`` is a name in ``share/maze_explorer/sim/mazes`` (without ``.txt``) or a path to a
maze text file (format in ``maze_explorer/maze_world.py``). The robot starts at the ``S``
cell, which is the world origin, facing +x.

Use another ROS domain than the robot's (4), e.g. ``ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1``,
so the simulated ``/cmd_vel`` can't reach a real robot.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from maze_explorer.maze_world import maze_to_sdf, parse_maze

# Static mounts from robot_info.md §5 (on the robot they come from /tf_static).
LIDAR_TF = ['0.18678', '0', '0.12498', '0', '0.1305', '0', '0.9914', 'base_link', 'livox_frame']
CAMERA_TF = ['0.25228', '-0.0005', '0.0741', '0', '0.1736', '0', '0.9848', 'base_link',
             'camera_link']
PASS_THROUGH = ['scan', 'slam', 'nav2', 'relay', 'explore', 'supervisor', 'spin', 'coverage',
                'debug', 'gait', 'gait_min_lift', 'gait_check_fail', 'entrance',
                'fence_polygon']
# Defaults that differ from the robot: no bridge in the simulation, so no gait change.
SIM_DEFAULTS = {'debug': 'false', 'gait': 'none', 'gait_min_lift': '0.0',
                'gait_check_fail': 'warn', 'entrance': '', 'fence_polygon': ''}


def _world(context):
    """Write the maze world to a temp file and start gzserver (and gzclient) with it."""
    share = get_package_share_directory('maze_explorer')
    maze = LaunchConfiguration('maze').perform(context)
    path = maze if os.path.sep in maze or maze.endswith('.txt') else os.path.join(
        share, 'sim', 'mazes', maze + '.txt')
    with open(path) as f:
        sdf = maze_to_sdf(parse_maze(f.read()))
    world = os.path.join(tempfile.mkdtemp(prefix='maze_world_'), 'maze.world')
    with open(world, 'w') as f:
        f.write(sdf)
    gazebo = get_package_share_directory('gazebo_ros')
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world, 'verbose': 'false'}.items())]
    if LaunchConfiguration('gui').perform(context).lower() == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch',
                                                       'gzclient.launch.py'))))
    return actions


def generate_launch_description():
    """Build the launch description."""
    share = get_package_share_directory('maze_explorer')
    lc = LaunchConfiguration
    args = [
        DeclareLaunchArgument('maze', default_value='example_maze',
                              description='maze name in sim/mazes or path to a .txt'),
        DeclareLaunchArgument('gui', default_value='false', description='open gzclient'),
        DeclareLaunchArgument('rviz', default_value='false', description='open RViz'),
    ] + [DeclareLaunchArgument(a, default_value=SIM_DEFAULTS.get(a, 'true'))
         for a in PASS_THROUGH]
    sim = {'use_sim_time': True}
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-entity', 'lite3', '-file', os.path.join(share, 'sim', 'lite3_sim.sdf'),
                   '-x', '0', '-y', '0', '-z', '0.36', '-timeout', '120'])
    tfs = [Node(package='tf2_ros', executable='static_transform_publisher',
                name=f'tf_{a[-1]}', arguments=a, parameters=[sim])
           for a in (LIDAR_TF, CAMERA_TF)]
    explore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'maze_explore.launch.py')),
        launch_arguments=[('use_sim_time', 'true')] + [(a, lc(a)) for a in PASS_THROUGH])
    rviz = ExecuteProcess(
        cmd=['rviz2', '-d', os.path.join(share, 'rviz', 'maze_explore.rviz'),
             '--ros-args', '-p', 'use_sim_time:=true'],
        condition=IfCondition(lc('rviz')))
    return LaunchDescription(args + [OpaqueFunction(function=_world), spawn] + tfs
                             + [explore, rviz])
