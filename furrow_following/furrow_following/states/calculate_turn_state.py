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
        self.start_yaw = None
        self.overshoot_correction = False

        # Angular velocity controller parameters
        self.kp = 0.04
        self.max_angular_speed = 1.5
        self.min_angular_speed = 0.5

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

        if self.start_yaw is None:
            self.start_yaw = current_yaw
            self.overshoot_correction = False
            yasmin.YASMIN_LOG_INFO(f"Start yaw: {math.degrees(current_yaw):.2f} deg")
            return CONTINUES

        angle_turned = self.normalize_angle(current_yaw - self.start_yaw)
        degrees_turned = math.degrees(angle_turned)
        yasmin.YASMIN_LOG_INFO(f"Turned: {degrees_turned:.2f} deg")

        error = self.target_angle - degrees_turned

        if abs(error) > 1.0:
            # Proportional control for angular velocity
            angular_speed = self.kp * abs(error)
            angular_speed = max(
                self.min_angular_speed, min(self.max_angular_speed, angular_speed)
            )
            blackboard["twist_msg"].angular.z = math.copysign(angular_speed, error)
            return CONTINUES
        elif not self.overshoot_correction and abs(error) <= 1.0:
            # Target nearly reached, stop movement
            self.overshoot_correction = True
            yasmin.YASMIN_LOG_INFO("Angle reached, stopping rotation.")
            return CONTINUES

        # Finalize turn
        self.start_yaw = None
        return ENDS
