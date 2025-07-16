import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    realsense_launch_dir = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch"
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_launch_dir, "rs_launch.py")),
        launch_arguments={
            "rgb_camera.color_profile": "1280,720,30",
            "depth_module.depth_profile": "1280,720,30",
            "align_depth.enable": "true",
        }.items(),
    )

    depth_proc_container = ComposableNodeContainer(
        name="depth_proc_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            # Register depth to RGB frame
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::RegisterNode",
                name="register_node",
                remappings=[
                    ("depth/image_rect", "/camera/camera/depth/image_rect_raw"),
                    ("rgb/camera_info", "/camera/camera/color/camera_info"),
                    ("depth/camera_info", "/camera/camera/depth/camera_info"),
                    (
                        "depth_registered/image_rect",
                        "/camera/camera/depth_registered/image_rect",
                    ),
                    (
                        "depth_registered/camera_info",
                        "/camera/camera/depth_registered/camera_info",
                    ),
                ],
            ),
            # Convert to 32FC1
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::ConvertMetricNode",
                name="convert_metric_node",
                remappings=[
                    ("image_raw", "/camera/camera/depth_registered/image_rect"),
                    ("image", "/camera/camera/depth_registered/image_rect_32fc1"),
                ],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([realsense_launch, depth_proc_container])
