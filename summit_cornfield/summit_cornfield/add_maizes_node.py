#!/usr/bin/env python3

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


import random
import time
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory


class MaizeSpawner(Node):
    def __init__(self):
        super().__init__("corn_spawner")

        # Declare and get parameters
        self.declare_parameter("init_x", -4.6)
        self.declare_parameter("init_y", -4.8)
        self.declare_parameter("rows", 20)
        self.declare_parameter("cols", 50)
        self.declare_parameter("spacing_x", 0.5)
        self.declare_parameter("spacing_x_reduction", 0.99925)
        self.declare_parameter("spacing_y", 0.195)
        self.declare_parameter("spacing_y_reduction", 1.0)

        self.init_x = self.get_parameter("init_x").value
        self.init_y = self.get_parameter("init_y").value
        self.rows = self.get_parameter("rows").value
        self.cols = self.get_parameter("cols").value
        self.spacing_x = self.get_parameter("spacing_x").value
        self.spacing_x_reduction = self.get_parameter("spacing_x_reduction").value
        self.spacing_y = self.get_parameter("spacing_y").value
        self.spacing_y_reduction = self.get_parameter("spacing_y_reduction").value

        # Load xml model file
        self.corn_model_xml = open(
            get_package_share_directory("summit_cornfield") + "/models/corn_3/model.sdf"
        ).read()

        # Create a service client
        self.client = self.create_client(SpawnEntity, "/spawn_entity")

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("Waiting for /spawn_entity service...")

        self.spawn_corn_grid()

    def spawn_corn_grid(self):
        """Spawns corns in a grid pattern using nested loops."""

        for i in range(self.rows):
            for j in range(self.cols):
                x = self.init_x + i * self.spacing_x
                y = self.init_y + j * self.spacing_y
                self.spawn_corn(x, y)
                time.sleep(0.01)  # Delay to prevent service overload

            self.spacing_x *= self.spacing_x_reduction
            self.spacing_y *= self.spacing_y_reduction

    def spawn_corn(self, x, y):
        """Calls the /spawn_entity service to spawn a corn at (x, y)."""

        request = SpawnEntity.Request()
        request.name = f"maize_{x}_{y}"
        request.xml = self.corn_model_xml
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = 0.05
        request.initial_pose.orientation.x = 0.0
        request.initial_pose.orientation.y = 0.0
        request.initial_pose.orientation.z = 0.0
        request.initial_pose.orientation.w = 1.0

        # Generate a random orientation in z and w
        request.initial_pose.orientation.z = random.uniform(0, 1)
        request.initial_pose.orientation.w = 1 - request.initial_pose.orientation.z

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Successfully spawned corn at ({x}, {y})")
        else:
            self.get_logger().error(f"Failed to spawn corn at ({x}, {y})")


def main(args=None):
    rclpy.init(args=args)
    node = MaizeSpawner()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
