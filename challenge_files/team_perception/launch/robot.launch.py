"""
Start the robot perception pipeline: qr_scanner, human_detector and poster_detector.

    ros2 launch team_perception robot.launch.py
    ros2 launch team_perception robot.launch.py depth_topic:=/camera/aligned_depth/image_raw
    ros2 launch team_perception robot.launch.py posters:=false   # QR scanner only

Every argument has a default that matches the robot's Orbbec camera. Check the real topic
names with `ros2 topic list` before a run. poster_detector needs depth ALIGNED to the
colour image, localisation (TF from the camera to map_frame) and the static map.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = {
    'image_topic': ('/camera/color/image_raw', 'colour image topic (raw name)'),
    'image_transport': ('compressed', "'raw', or 'compressed' = <image_topic>/compressed"),
    'depth_topic': ('/camera/depth/image_raw', 'depth image, aligned to the colour image'),
    'camera_info_topic': ('/camera/color/camera_info', 'colour camera intrinsics'),
    'use_depth': ('true', 'place QR codes and people in the map using depth'),
    'map_topic': ('/map', 'static map (nav_msgs/OccupancyGrid, transient local)'),
    'map_frame': ('map', 'frame that positions are reported in'),
    'map_mode': ('veto', "poster_detector map test: 'require', 'veto' or 'off'"),
    'qr': ('true', 'run qr_scanner'),
    'posters': ('true', 'run human_detector + poster_detector'),
}


def generate_launch_description():
    c = {name: LaunchConfiguration(name) for name in ARGS}
    camera = {'image_topic': c['image_topic'], 'image_transport': c['image_transport'],
              'depth_topic': c['depth_topic'], 'camera_info_topic': c['camera_info_topic'],
              'use_depth': c['use_depth'], 'map_frame': c['map_frame']}
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=desc)
          for name, (default, desc) in ARGS.items()],
        Node(package='team_perception', executable='qr_scanner', name='qr_scanner',
             output='screen', parameters=[camera], condition=IfCondition(c['qr'])),
        Node(package='team_perception', executable='human_detector', name='human_detector',
             output='screen', parameters=[camera], condition=IfCondition(c['posters'])),
        Node(package='team_perception', executable='poster_detector', name='poster_detector',
             output='screen',
             parameters=[{'depth_topic': c['depth_topic'],
                          'camera_info_topic': c['camera_info_topic'],
                          'map_topic': c['map_topic'], 'map_frame': c['map_frame'],
                          'map_mode': c['map_mode']}],
             condition=IfCondition(c['posters'])),
    ])
