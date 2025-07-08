from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

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

    ld = LaunchDescription()

    ld.add_action(depth_image_centering_node)
    ld.add_action(furrow_following_node)

    return ld
