"""
Start the onboard half of the perception pipeline: qr_scanner, poster_detector, image_throttle.

    ros2 launch team_perception robot.launch.py
    ros2 launch team_perception robot.launch.py depth_topic:=/camera/aligned_depth/image_raw \
        depth_camera_info_topic:=''        # depth already aligned by the driver
    ros2 launch team_perception robot.launch.py posters:=false   # QR scanner only

YOLO person detection (human_detector) runs on the EXTERNAL computer
(`ros2 launch team_perception_external external.launch.py`). image_throttle sends it a
reduced-rate JPEG stream; it returns /humans/detections, which poster_detector judges here
against the local depth image. Raw depth and point clouds never leave the robot.

Every argument has a default that matches the robot's Orbbec camera. Check the real topic
names with `ros2 topic list` before a run. poster_detector needs depth ALIGNED to the
colour image or depth_camera_info_topic (the Lite3's depth is NOT aligned; the nodes then
map colour pixels into it, a temporary fix), localisation (TF from the camera to map_frame)
and the static map.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = {
    'image_topic': ('/camera/color/image_raw', 'colour image topic (raw name)'),
    'image_transport': ('compressed', "'raw', or 'compressed' = <image_topic>/compressed"),
    'depth_topic': ('/camera/depth/image_raw', 'depth image (aligned to colour or not)'),
    'depth_camera_info_topic': ('/camera/depth/camera_info',
                                "depth intrinsics, to map colour pixels into unaligned depth; "
                                "'' = depth_topic is already aligned to colour"),
    'camera_info_topic': ('/camera/color/camera_info', 'colour camera intrinsics'),
    'use_depth': ('true', 'place QR codes in the map using depth'),
    'map_topic': ('/map', 'static map (nav_msgs/OccupancyGrid, transient local)'),
    'map_frame': ('map', 'frame that positions are reported in'),
    'map_mode': ('veto', "poster_detector map test: 'require', 'veto' or 'off'"),
    'offboard_topic': ('/camera/color/image_offboard/compressed',
                       'throttled JPEG stream for the external computer'),
    'offboard_rate_hz': ('10.0', 'frames per second sent to the external computer'),
    'qr': ('true', 'run qr_scanner'),
    'posters': ('true', 'run poster_detector + image_throttle (needs the external computer)'),
}


def generate_launch_description():
    c = {name: LaunchConfiguration(name) for name in ARGS}
    camera = {'image_topic': c['image_topic'], 'image_transport': c['image_transport'],
              'depth_topic': c['depth_topic'], 'camera_info_topic': c['camera_info_topic'],
              'depth_camera_info_topic': c['depth_camera_info_topic'],
              'use_depth': c['use_depth'], 'map_frame': c['map_frame']}
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=desc)
          for name, (default, desc) in ARGS.items()],
        Node(package='team_perception', executable='qr_scanner', name='qr_scanner',
             output='screen', parameters=[camera], condition=IfCondition(c['qr'])),
        Node(package='team_perception', executable='image_throttle', name='image_throttle',
             output='screen',
             parameters=[{'in_topic': [c['image_topic'], '/compressed'],
                          'out_topic': c['offboard_topic'],
                          'rate_hz': c['offboard_rate_hz']}],
             condition=IfCondition(c['posters'])),
        Node(package='team_perception', executable='poster_detector', name='poster_detector',
             output='screen',
             parameters=[{'depth_topic': c['depth_topic'],
                          'depth_camera_info_topic': c['depth_camera_info_topic'],
                          'camera_info_topic': c['camera_info_topic'],
                          'map_topic': c['map_topic'], 'map_frame': c['map_frame'],
                          'map_mode': c['map_mode']}],
             condition=IfCondition(c['posters'])),
    ])
