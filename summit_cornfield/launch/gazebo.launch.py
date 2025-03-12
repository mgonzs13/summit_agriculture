# MIT License
#
# Copyright (c) 2025 Miguel Ángel González Santamarta
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():

    pkg_path = get_package_share_directory("summit_cornfield")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    world = LaunchConfiguration("world")
    world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg_path, "worlds", "empty.world"),
        description="Gazebo world",
    )

    launch_gui = LaunchConfiguration("launch_gui")
    launch_gui_cmd = DeclareLaunchArgument(
        "launch_gui",
        default_value="True",
        description="Whether launch gzclient",
    )

    pause_gz = LaunchConfiguration("pause_gz")
    pause_gz_cmd = DeclareLaunchArgument(
        "pause_gz",
        default_value="False",
        description="Whether to pause gazebo",
    )

    gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        ),
        condition=IfCondition(PythonExpression([launch_gui])),
    )

    gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "world": world,
            "pause": pause_gz,
        }.items(),
    )

    summit_spawn_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("summit_xl_gazebo"),
                "launch",
                "spawn.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "robot_id": "robot",
            "namespace": "robot",
            "pos_x": "-11.0",
            "pos_y": "-11.5",
        }.items(),
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
            "use_sim_time": "True",
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
            "use_sim_time": "true",
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(launch_gui_cmd)
    ld.add_action(pause_gz_cmd)
    ld.add_action(world_cmd)

    ld.add_action(summit_localization_cmd)
    ld.add_action(summit_navigation_cmd)

    ld.add_action(gazebo_client_cmd)
    ld.add_action(gazebo_server_cmd)
    ld.add_action(summit_spawn_cmd)

    return ld
