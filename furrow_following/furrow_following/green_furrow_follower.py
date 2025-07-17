#!/usr/bin/env python3
# filepath: /home/robotica/maizada_ws/src/summit_agriculture/maize_gps_viewer/test.py
"""
Green Line Follower Node for agricultural robots.
This ROS2 node detects and follows green lines (crop rows) using computer vision.
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class LineFollowerNode(Node):
    """
    A ROS2 node that detects and follows green lines using a camera.
    Publishes visualization images and velocity commands.
    """

    def __init__(self):
        """Initialize the line follower node with parameters and publishers/subscribers."""
        super().__init__("line_follower_node")

        # Initialize the CV bridge for image conversion
        self.bridge = CvBridge()

        # Create subscribers and publishers
        self.image_sub = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.image_callback, 10
        )
        self.cmd_pub = self.create_publisher(
            Twist, "/robot/robotnik_base_control/cmd_vel", 10
        )
        self.visual_pub = self.create_publisher(Image, "/line_follower/visualization", 10)

        # Set control frequency to 20Hz for better responsiveness
        self.timer = self.create_timer(0.05, self.control_loop)

        # State variables
        self.cx = None  # Current detected line center x position
        self.image_width = 640

        # Declare adjustable control parameters
        self.declare_parameter("linear_speed", 1.2)
        self.declare_parameter("angular_gain", 0.02)
        self.declare_parameter("search_speed", 0.6)

    def image_callback(self, msg):
        """
        Process camera images to detect green lines.

        Args:
            msg: ROS Image message from camera
        """
        # Convert ROS image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # Save a copy for visualization
        visual_frame = frame.copy()

        # Mark region of interest on the visual image
        roi_y = frame.shape[0] // 2
        cv2.line(visual_frame, (0, roi_y), (frame.shape[1], roi_y), (0, 255, 255), 2)

        # Extract region of interest (lower half of the image)
        roi = frame[roi_y:, :]

        # Convert to HSV for better color segmentation
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Define green color range
        lower_green = np.array([35, 80, 40])
        upper_green = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Apply morphological operations to improve detection
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Visualize the mask in the bottom part of the image
        visual_roi = visual_frame[roi_y:, :]
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        # Overlay semi-transparent mask
        cv2.addWeighted(mask_colored, 0.5, visual_roi, 0.5, 0, visual_roi)

        center_x = self.image_width // 2

        # Process detected contours
        if contours:
            # Filter out very small contours (noise)
            valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 100]

            if valid_contours:
                # For each contour, calculate its center and distance to image center
                contour_centers = []
                for cnt in valid_contours:
                    M = cv2.moments(cnt)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        # Calculate distance to center
                        distance = abs(cx - center_x)
                        contour_centers.append((cx, cy, distance, cnt))

                # Sort by distance to center
                contour_centers.sort(key=lambda x: x[2])

                # Select the contour closest to center
                if contour_centers:
                    best_cx, best_cy, best_distance, best_contour = contour_centers[0]

                    # Draw all contours in the visualization
                    cv2.drawContours(visual_roi, valid_contours, -1, (0, 255, 0), 2)

                    # Highlight the selected contour
                    cv2.drawContours(visual_roi, [best_contour], -1, (0, 255, 255), 3)

                    # Update the center point for control
                    self.cx = best_cx

                    # Draw detected center point (adjusting coordinates to full image)
                    cv2.circle(
                        visual_frame, (best_cx, roi_y + best_cy), 10, (0, 0, 255), -1
                    )

                    # Draw line from center to detected point
                    cv2.line(
                        visual_frame,
                        (center_x, roi_y + best_cy),
                        (best_cx, roi_y + best_cy),
                        (255, 0, 0),
                        2,
                    )

                    # Display the error
                    error = best_cx - center_x
                    cv2.putText(
                        visual_frame,
                        f"Error: {error}",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 0, 255),
                        2,
                    )
                    cv2.putText(
                        visual_frame,
                        f"Following central line",
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2,
                    )
                else:
                    self.cx = None
            else:
                self.cx = None
        else:
            self.cx = None

        if self.cx is None:
            cv2.putText(
                visual_frame,
                "No line detected",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2,
            )

        # Publish visualization image
        visual_msg = self.bridge.cv2_to_imgmsg(visual_frame, encoding="bgr8")
        self.visual_pub.publish(visual_msg)

    def control_loop(self):
        """Calculate and publish control commands based on line detection."""
        twist = Twist()

        # Get parameters from ROS
        linear_speed = self.get_parameter("linear_speed").value
        angular_gain = self.get_parameter("angular_gain").value
        search_speed = self.get_parameter("search_speed").value

        if self.cx is not None:
            # Line is detected - calculate error and adjust steering
            error = self.cx - self.image_width // 2
            twist.linear.x = linear_speed
            twist.angular.z = -float(error) * angular_gain
        else:
            # No line detected - search pattern
            twist.linear.x = 0.2
            twist.angular.z = search_speed

        # Publish command velocity
        self.cmd_pub.publish(twist)


def main(args=None):
    """Main entry point for the line follower node."""
    rclpy.init(args=args)
    node = LineFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
