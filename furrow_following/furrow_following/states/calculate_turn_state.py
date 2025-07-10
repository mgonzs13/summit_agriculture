import math
import yasmin
from yasmin import State
from yasmin.blackboard import Blackboard
from geometry_msgs.msg import Twist
import tf_transformations

from furrow_following.states.outcomes import ENDS, CONTINUES


class CalculateTurnState(State):

    def __init__(self, target_angle: float = -90.0) -> None:
        super().__init__([CONTINUES, ENDS])
        self.target_angle = target_angle  # degrees

    def get_yaw_from_quaternion(self, quat) -> float:
        _, _, yaw = tf_transformations.euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w]
        )
        return yaw

    def normalize_angle(self, angle) -> float:
        """Normalize angle to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))

    def execute(self, blackboard: Blackboard) -> str:
        msg = blackboard["odom_msg"]
        current_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)

        blackboard["twist_msg"] = Twist()

        if "start_yaw" not in blackboard or blackboard["start_yaw"] is None:
            blackboard["start_yaw"] = current_yaw
            yasmin.YASMIN_LOG_INFO(f"Start yaw: {math.degrees(current_yaw):.2f} deg")
            return CONTINUES

        angle_turned = self.normalize_angle(current_yaw - blackboard["start_yaw"])
        degrees_turned = math.degrees(angle_turned)
        yasmin.YASMIN_LOG_INFO(f"Turned: {degrees_turned:.2f} deg")

        # Check if desired angle (positive or negative) has been reached
        if (self.target_angle > 0 and degrees_turned < self.target_angle) or (
            self.target_angle < 0 and degrees_turned > self.target_angle
        ):
            blackboard["twist_msg"].angular.z = 1.0 if self.target_angle > 0 else -1.0
            return CONTINUES

        return ENDS
