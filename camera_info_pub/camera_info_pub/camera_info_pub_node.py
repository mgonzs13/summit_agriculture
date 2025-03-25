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


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__("camera_info_publisher")

        self.subscription = self.create_subscription(
            Image, "/robot/zed2/zed_node/rgb/image_rect_color", self.image_callback, 30
        )

        self.publisher = self.create_publisher(
            CameraInfo, "/robot/zed2/zed_node/camera_info", 30
        )
        self.get_logger().info("Starting camera_info publishing")

    def image_callback(self, msg: Image):
        camera_info = CameraInfo()
        camera_info.header = msg.header
        camera_info.height = 376
        camera_info.width = 672
        camera_info.distortion_model = "plumb_bob"
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.k = [
            260.9255065917969,
            0.0,
            335.02545166015625,
            0.0,
            260.9255065917969,
            190.65074157714844,
            0.0,
            0.0,
            1.0,
        ]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [
            260.9255065917969,
            0.0,
            335.02545166015625,
            0.0,
            0.0,
            260.9255065917969,
            190.65074157714844,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        camera_info.binning_x = 0
        camera_info.binning_y = 0
        camera_info.roi.x_offset = 0
        camera_info.roi.y_offset = 0
        camera_info.roi.height = 0
        camera_info.roi.width = 0
        camera_info.roi.do_rectify = False

        self.publisher.publish(camera_info)
        self.get_logger().info("Published CameraInfo message")


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
