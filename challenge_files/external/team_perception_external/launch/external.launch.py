"""
Start the external-computer half of the perception pipeline: human_detector (YOLO).

    ros2 launch team_perception_external external.launch.py
    ros2 launch team_perception_external external.launch.py device:=0      # NVIDIA GPU
    ros2 launch team_perception_external external.launch.py rate_hz:=15.0 imgsz:=480

Run it on a faster computer on the robot's network with the robot's ROS_DOMAIN_ID (4 on the
Lite3). It reads the throttled JPEG stream that the robot's image_throttle publishes and sends
back /humans/detections (small JSON), which poster_detector on the robot judges against the
depth image. Depth is NOT streamed to this computer by default (too much bandwidth), so the
person positions in /humans/detections are null; poster_detector computes poster positions
itself. Set use_depth:=true only on a wired link.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = {
    'image_topic': ('/camera/color/image_offboard',
                    'colour image topic (raw name; /compressed is added)'),
    'image_transport': ('compressed', "'raw', or 'compressed' = <image_topic>/compressed"),
    'use_depth': ('false', 'person positions from depth (streams raw depth: wired link only)'),
    'depth_topic': ('/camera/depth/image_raw', 'depth image, if use_depth'),
    'depth_camera_info_topic': ('/camera/depth/camera_info', 'depth intrinsics, if use_depth'),
    'camera_info_topic': ('/camera/color/camera_info', 'colour intrinsics, if use_depth'),
    'map_frame': ('map', 'frame for person positions, if use_depth'),
    'model': ('yolo11n.pt', 'YOLO weights: bare name -> ~/.cache/team_perception/, or a path'),
    'conf': ('0.4', 'minimum person confidence'),
    'imgsz': ('640', 'YOLO input size'),
    'device': ('cpu', "'cpu', or '0' for the first CUDA GPU"),
    'rate_hz': ('10.0', 'max inferences per second'),
}


def generate_launch_description():
    c = {name: LaunchConfiguration(name) for name in ARGS}
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=desc)
          for name, (default, desc) in ARGS.items()],
        Node(package='team_perception_external', executable='human_detector',
             name='human_detector', output='screen', parameters=[c]),
    ])
