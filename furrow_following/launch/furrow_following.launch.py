import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if True",
    )

    is_sim = LaunchConfiguration("is_sim")
    is_sim_cmd = DeclareLaunchArgument(
        "is_sim",
        default_value="True",
        description="Whether use sim configuration",
    )

    depth_image_centering_node = Node(
        package="furrow_following",
        executable="depth_image_centering_node",
        name="depth_image_centering_node",
    )

    furrow_following_node = Node(
        package="furrow_following",
        executable="furrow_following_node",
        name="furrow_following_node",
    )

    summit_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("summit_localization"),
                "launch",
                "localization.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "is_sim": is_sim,
            "use_gps": "True",
            "use_rtabmap_cmd": "False",
            "use_obstacles_detection": "False",
        }.items(),
    )

    summit_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("summit_navigation"),
                "launch",
                "bringup.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_cmd)
    ld.add_action(is_sim_cmd)
    ld.add_action(depth_image_centering_node)
    ld.add_action(furrow_following_node)
    ld.add_action(summit_localization_cmd)
    # ld.add_action(summit_navigation_cmd)

    return ld
