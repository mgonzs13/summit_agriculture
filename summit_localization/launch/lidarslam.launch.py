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
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    lidarslam_params_file = launch.substitutions.LaunchConfiguration(
        "lidarslam_params_file",
        default=os.path.join(
            get_package_share_directory("summit_localization"), "config", "lidarslam.yaml"
        ),
    )

    mapping = launch_ros.actions.Node(
        package="scanmatcher",
        executable="scanmatcher_node",
        parameters=[lidarslam_params_file],
        remappings=[
            ("/input_cloud", "/robot/top_3d_laser/points_filtered"),
            ("imu", "/robot/imu/data"),
        ],
        output="screen",
    )

    graphbasedslam = launch_ros.actions.Node(
        package="graph_based_slam",
        executable="graph_based_slam_node",
        parameters=[lidarslam_params_file],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                "lidarslam_params_file",
                default_value=lidarslam_params_file,
                description="Full path to main parameter file to load",
            ),
            mapping,
            graphbasedslam,
        ]
    )
