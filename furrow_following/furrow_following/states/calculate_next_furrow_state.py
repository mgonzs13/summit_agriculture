import math
import tf_transformations
from yasmin import State
from yasmin.blackboard import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose


class CalculateNextFurrowState(State):

    def __init__(self, furrow_distance: float = 0.5) -> None:
        super().__init__([SUCCEED])
        self.furrow_distance = furrow_distance

    def execute(self, blackboard: Blackboard) -> str:
        msg: Odometry = blackboard["odom_msg"]

        position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation

        # Convert quaternion to Euler angles
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )

        # Compute displacement: 0.5 meters to the right in robot's local frame (negative Y)
        dx = 0.5 * math.sin(yaw)
        dy = -0.5 * math.cos(yaw)

        new_x = position.x + dx
        new_y = position.y + dy
        new_z = position.z  # Assuming same Z height

        # Invert the yaw angle (rotate by 180 degrees)
        new_yaw = yaw + math.pi
        new_yaw = (new_yaw + math.pi) % (
            2 * math.pi
        ) - math.pi  # Normalize between [-pi, pi]

        # Convert back to quaternion
        new_orientation_q = tf_transformations.quaternion_from_euler(0.0, 0.0, new_yaw)

        # Create and publish new PoseStamped
        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose.header = msg.header
        nav2_goal.pose.header.frame_id = "map"
        nav2_goal.pose.pose.position.x = new_x
        nav2_goal.pose.pose.position.y = new_y
        nav2_goal.pose.pose.position.z = new_z
        nav2_goal.pose.pose.orientation.x = new_orientation_q[0]
        nav2_goal.pose.pose.orientation.y = new_orientation_q[1]
        nav2_goal.pose.pose.orientation.z = new_orientation_q[2]
        nav2_goal.pose.pose.orientation.w = new_orientation_q[3]

        blackboard["nav2_goal"] = nav2_goal

        return SUCCEED
