from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rgbd",
            executable="ros_to_rgbd",
            name="rgbd_encoder",
            output="screen",
            remappings=[
                ("cam_info", "/camera/rgb/camera_info"),
                ("rgb_image", "/camera/rgb/image_rect_color"),
                ("depth_image", "/camera/depth_registered/hw_registered/image_rect_raw"),
                ("rgbd", "/camera/rgbd"),
            ],
        ),
    ])
