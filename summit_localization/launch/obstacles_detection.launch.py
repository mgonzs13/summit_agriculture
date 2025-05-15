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


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True",
    )

    parameters = [
        {
            "map_frame_id": "map",
            "frame_id": "robot/base_footprint",
            "subscribe_depth": True,
            "subscribe_rgb": True,
            "subscribe_scan_cloud": True,
            "approx_sync": True,
            "sync_queue_size": 30,
            "use_sim_time": use_sim_time,
            "qos": 2,
            "qos_image": 1,
            "qos_camera_info": 1,
            "Grid/RangeMin": "0.0",
            "Grid/RangeMax": "5.0",
            "Grid/MaxObstacleHeight": "1.0",
        }
    ]

    return LaunchDescription(
        [
            use_sim_time_cmd,
            Node(
                package="rtabmap_util",
                executable="obstacles_detection",
                name="lidar3d_obstacles_detection",
                output="log",
                parameters=parameters,
                remappings=[
                    ("cloud", "/robot/top_3d_laser/points_filtered"),
                    ("obstacles", "/lidar3d_obstacles"),
                    ("ground", "/lidar3d_ground"),
                ],
                arguments=["--ros-args", "--log-level", "Warn"],
            ),
            Node(
                package="rtabmap_util",
                executable="obstacles_detection",
                name="rgbd_obstacles_detection",
                output="log",
                parameters=parameters,
                remappings=[
                    ("cloud", "/robot/zed2/zed_node/point_cloud/cloud_registered"),
                    ("obstacles", "/rgbd_obstacles"),
                    ("ground", "/rgbd_ground"),
                ],
                arguments=["--ros-args", "--log-level", "Warn"],
            ),
        ]
    )
