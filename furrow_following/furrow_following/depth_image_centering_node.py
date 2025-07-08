#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped


class DepthImageCenteringNode(Node):
    def __init__(self):
        super().__init__("depth_image_centering_node")

        # Parameters
        self.declare_parameter(
            "depth_topic", "/robot/zed2/zed_node/depth/depth_registered"
        )
        self.declare_parameter("camera_info_topic", "/robot/zed2/zed_node/camera_info")
        self.declare_parameter("target_frame", "robot/base_footprint")
        self.declare_parameter("output_topic", "/depth_centered")

        self.depth_topic = (
            self.get_parameter("depth_topic").get_parameter_value().string_value
        )
        self.camera_info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )
        self.output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )

        self.bridge = CvBridge()

        # TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera intrinsics
        self.K = None  # Camera intrinsic matrix

        # Subscribers
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_image_callback, 10
        )

        # Publisher
        self.image_pub = self.create_publisher(Image, self.output_topic, 10)

    def camera_info_callback(self, msg: CameraInfo):
        # Cache the intrinsics matrix
        self.K = np.array(msg.k).reshape(3, 3)
        self.get_logger().info("Received camera intrinsics.")
        # Unsubscribe from camera info after first receipt
        self.destroy_subscription(self.camera_info_sub)

    def depth_image_callback(self, msg: Image) -> None:
        if self.K is None:
            self.get_logger().warn("Waiting for camera intrinsics...")
            return

        try:
            # Transform lookup
            src_frame = msg.header.frame_id
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                src_frame, self.target_frame, rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"Transform unavailable: {str(e)}")
            return

        # Convert ROS Image to OpenCV format
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        if depth_image.ndim != 2:
            self.get_logger().error("Expected a single-channel depth image.")
            return

        height, width = depth_image.shape

        # Get translation vector from base_footprint to camera
        dx = transform.transform.translation.x
        dz = transform.transform.translation.z

        # Project translation into image space
        u_offset = int(-dx * self.K[0, 0] / dz)

        # Calculate cropping bounds
        if u_offset > 0:
            cropped_image = depth_image[:, u_offset:]
        elif u_offset < 0:
            cropped_image = depth_image[:, :u_offset]
        else:
            cropped_image = depth_image.copy()

        # Adjust size back to original width (optional)
        cropped_width = cropped_image.shape[1]
        final_width = min(width, cropped_width)
        cropped_image = cropped_image[:, :final_width]

        # Publish the new image
        new_msg = self.bridge.cv2_to_imgmsg(cropped_image, encoding=msg.encoding)
        new_msg.header = msg.header
        new_msg.header.frame_id = self.target_frame  # Update to indicate centering frame
        self.image_pub.publish(new_msg)
        self.get_logger().debug("Published cropped and centered depth image.")


def main(args=None):
    rclpy.init(args=args)
    node = DepthImageCenteringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
