# MIT License

# Copyright (c) 2025 Miguel Ángel González Santamarta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():

    pkg_localization = get_package_share_directory("summit_localization")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True",
    )

    slam_mode = LaunchConfiguration("slam_mode")
    slam_mode_cmd = DeclareLaunchArgument(
        "slam_mode",
        default_value="True",
        description="Whether to run in SLAM mode. Otherwise, it will run in Localization mode",
    )

    is_sim = LaunchConfiguration("is_sim")
    is_sim_cmd = DeclareLaunchArgument(
        "is_sim",
        default_value="False",
        description="Whether use sim configuration",
    )

    use_gps = LaunchConfiguration("use_gps")
    use_gps_cmd = DeclareLaunchArgument(
        "use_gps",
        default_value="False",
        description="Whether use GPS in localization",
    )

    laser_filter_node_cmd = Node(
        package="robotnik_filters",
        executable="filter_node",
        name="filter_node",
        remappings=[
            ("~/input", "/robot/top_3d_laser/points"),
            ("~/output", "/robot/top_3d_laser/points_filtered"),
        ],
        condition=IfCondition(PythonExpression([is_sim])),
    )

    camera_info_pub_cmd = Node(
        package="camera_info_pub",
        executable="camera_info_pub_node",
        name="camera_info_pub_node",
        condition=UnlessCondition(PythonExpression([is_sim])),
    )

    rgbd_odometry_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, "launch", "rgbd_odometry.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(PythonExpression([is_sim])),
    )

    rtabmap_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, "launch", "rtabmap.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_mode": slam_mode,
            "use_gps_map": use_gps,
        }.items(),
    )

    ekf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, "launch", "ekf.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    dual_ekf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, "launch", "dual_ekf.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(PythonExpression([use_gps])),
    )

    imu_compass_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, "launch", "imu_compass.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(PythonExpression([use_gps])),
    )

    set_compass_pose_cmd = Node(
        package="summit_localization",
        executable="cameraset_compass_pose_info_pub_node",
        name="set_compass_pose_node",
        condition=IfCondition(PythonExpression([use_gps])),
    )

    waypoint_follower_cmd = Node(
        package="summit_localization",
        executable="interactive_waypoint_follower",
        name="waypoint_follower",
        output="screen",
        condition=IfCondition(PythonExpression([use_gps])),
    )

    ld = LaunchDescription()

    ld.add_action(use_sim_time_cmd)
    ld.add_action(slam_mode_cmd)
    ld.add_action(is_sim_cmd)
    ld.add_action(use_gps_cmd)
    ld.add_action(laser_filter_node_cmd)
    ld.add_action(camera_info_pub_cmd)
    ld.add_action(rgbd_odometry_cmd)
    ld.add_action(rtabmap_cmd)
    ld.add_action(ekf_cmd)
    ld.add_action(dual_ekf_cmd)
    ld.add_action(imu_compass_cmd)
    ld.add_action(set_compass_pose_cmd)
    ld.add_action(waypoint_follower_cmd)

    return ld
