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
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True",
    )

    launch_mapviz = LaunchConfiguration("launch_mapviz")
    launch_mapviz_cmd = DeclareLaunchArgument(
        "launch_mapviz",
        default_value="False",
        description="Whether to start mapviz",
    )

    params_file = os.path.join(
        get_package_share_directory("summit_localization"),
        "config",
        "dual_ekf_navsat.yaml",
    )

    param_substitutions = {"use_sim_time": use_sim_time}

    configured_params = RewrittenYaml(
        source_file=params_file, param_rewrites=param_substitutions, convert_types=True
    )

    local_ekf_cmd = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="log",
        parameters=[configured_params],
        remappings=[
            ("odometry/filtered", "/local_odom"),
            ("accel/filtered", "/local_accel"),
        ],
    )

    global_ekf_cmd = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="log",
        parameters=[configured_params],
        remappings=[
            ("odometry/filtered", "/global_odom"),
            ("accel/filtered", "/global_accel"),
        ],
    )

    navsat_cmd = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[configured_params],
        remappings=[
            # Subscriptions
            ("imu", "/robot/zed2/zed_node/imu/data"),
            ("gps/fix", "/robot/gps/fix"),
            ("odometry/filtered", "global_odom"),
            # Publishers
            ("odometry/gps", "gps_odom"),
            ("gps/filtered", "gps/filtered"),
        ],
    )

    mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.join(
                    get_package_share_directory("summit_localization"), "launch"
                ),
                "mapviz.launch.py",
            )
        ),
        condition=IfCondition(launch_mapviz),
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_cmd)
    ld.add_action(launch_mapviz_cmd)
    ld.add_action(local_ekf_cmd)
    ld.add_action(global_ekf_cmd)
    ld.add_action(navsat_cmd)
    ld.add_action(mapviz_cmd)
    return ld
