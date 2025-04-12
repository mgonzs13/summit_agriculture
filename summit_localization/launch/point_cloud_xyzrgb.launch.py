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
            "use_sim_time": use_sim_time,
            "approx_sync": True,
            "approx_sync_max_interval": 0.0,
            "sync_queue_size": 10,
            "decimation": 1,
            "filter_nans": False,
            "max_depth": 0.0,
            "min_depth": 0.0,
            "noise_filter_min_neighbors": 5,
            "noise_filter_radius": 0.0,
            "normal_k": 0,
            "normal_radius": 0.0,
            "qos": 0,
            "qos_camera_info": 0,
            "queue_size": -1,
            "topic_queue_size": 1,
            "roi_ratios": "",
            "voxel_size": 0.0,
        }
    ]

    return LaunchDescription(
        [
            use_sim_time_cmd,
            Node(
                package="rtabmap_util",
                executable="point_cloud_xyzrgb",
                parameters=parameters,
                output="screen",
                remappings=[
                    ("rgb/image", "/robot/zed2/zed_node/rgb/image_rect_color"),
                    ("rgb/camera_info", "/robot/zed2/zed_node/camera_info"),
                    ("depth/image", "/robot/zed2/zed_node/depth/depth_registered"),
                    ("cloud", "/robot/zed2/zed_node/point_cloud/cloud_registered"),
                ],
            ),
        ]
    )
