import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_localization.srv import SetPose


class ImuPoseSetter(Node):

    def __init__(self):
        super().__init__("imu_pose_setter")

        self.is_working = True
        self.is_setting_pose = False

        # Create subscriber to IMU data
        self.subscription = self.create_subscription(
            Imu, "/imu/data_compass", self.imu_callback, 10
        )

        # Create client for /set_pose service
        self.client = self.create_client(SetPose, "/ekf_map/set_pose")

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("/ekf_map/set_pose service not available, waiting...")

        self.get_logger().info("IMU Pose Setter Node is ready.")

    def imu_callback(self, msg: Imu):
        if self.is_setting_pose:
            self.get_logger().info("Already setting pose, ignoring new IMU data.")
            return

        # Extract orientation from IMU
        orientation = msg.orientation

        # Create the pose message
        pose_msg = PoseWithCovarianceStamped()
        # pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        # Set orientation from IMU
        pose_msg.pose.pose.orientation.z = orientation.z
        pose_msg.pose.pose.orientation.w = orientation.w

        # Set position to zero
        pose_msg.pose.pose.position.x = 0.0
        pose_msg.pose.pose.position.y = 0.0
        pose_msg.pose.pose.position.z = 0.0

        # Set default covariance (optional: you can fill it appropriately)
        pose_msg.pose.covariance = [0.0] * 36

        # Create and send the request
        request = SetPose.Request()
        request.pose = pose_msg

        self.is_setting_pose = True
        self.get_logger().info("Setting pose with IMU data...")
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info("SetPose service call succeeded.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        finally:
            self.is_working = False


def main(args=None):
    rclpy.init(args=args)
    node = ImuPoseSetter()

    while node.is_working:
        rclpy.spin_once(node)
        if not node.is_working:
            break
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
