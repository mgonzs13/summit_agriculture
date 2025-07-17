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

    description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("summit_realsense"),
                "launch",
                "robot_state_publisher.launch.py",
            )
        )
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_launch_dir, "rs_launch.py")),
        launch_arguments={
            "rgb_camera.color_profile": "1280,720,30",
            "depth_module.depth_profile": "1280,720,30",
            "align_depth.enable": "true",
            "unite_imu_method": "1",
            "enable_gyro": "true",
            "enable_accel": "true",
        }.items(),
    )

    depth_proc_container = ComposableNodeContainer(
        name="depth_proc_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            # Convert to 32FC1
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::ConvertMetricNode",
                name="convert_metric_node",
                remappings=[
                    ("image_raw", "/camera/camera/depth/image_rect_raw"),
                    ("image", "/camera/camera/depth/image_rect_raw_32fc1"),
                ],
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            description_cmd,
            realsense_launch,
            depth_proc_container,
        ]
    )
