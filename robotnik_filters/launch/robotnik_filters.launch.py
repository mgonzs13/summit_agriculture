#  Copyright (c) 2022 Jonas Mahler

#  This file is part of pcl_example.

#  pcl_example is free software: you can redistribute it and/or modify it under the terms 
#  of the GNU General Public License as published by the Free Software Foundation, 
#  either version 3 of the License, or (at your option) any later version.

#  pcl_example is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
#  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
#  See the GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License along 
#  with pcl_example. If not, see <https://www.gnu.org/licenses/>. 

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        PushRosNamespace('robot'),
        Node(
            package='robotnik_filters',
            executable='filter_node',
            name='filter_node',
            remappings=[
                ('~/input', '/rslidar_points'),
                ('~/output', '/rslidar_points_filtered')

            ]
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', '/rslidar_points_filtered'),
                ('scan', '/scan')
            ],
            parameters=[
                {
                    'min_height': -0.1,
                    'max_height': 0.1,
                    'angle_min': -3.14159265359,
                    'angle_max': 3.14159265359,
                    'angle_increment': 0.005,
                    'target_frame': 'robot/top_3d_laser_link',
                }
            ],
        ),
    ])
