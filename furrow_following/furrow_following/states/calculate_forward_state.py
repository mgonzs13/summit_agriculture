import math
import yasmin
from yasmin import State
from yasmin.blackboard import Blackboard
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from furrow_following.states.outcomes import ENDS, CONTINUES


class CalculateForwardState(State):
    def __init__(self, target_distance: float = 0.5) -> None:
        """
        Move forward a target distance in meters.
        """
        super().__init__([CONTINUES, ENDS])
        self.target_distance = target_distance
        self.start_position = None
        self.overshoot_correction = False

        # Linear velocity controller parameters
        self.kp = 0.5
        self.max_linear_speed = 0.5
        self.min_linear_speed = 0.1

    def euclidean_distance(self, x1, y1, x2, y2) -> float:
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def execute(self, blackboard: Blackboard) -> str:
        msg: Odometry = blackboard["odom_msg"]
        position = msg.pose.pose.position

        blackboard["twist_msg"] = Twist()

        # Store initial position
        if self.start_position is None:
            self.start_position = (position.x, position.y)
            self.overshoot_correction = False
            yasmin.YASMIN_LOG_INFO(
                f"Start position: x={position.x:.2f}, y={position.y:.2f}"
            )
            return CONTINUES

        start_x, start_y = self.start_position
        distance_moved = self.euclidean_distance(start_x, start_y, position.x, position.y)
        yasmin.YASMIN_LOG_INFO(f"Moved: {distance_moved:.2f} m")

        error = self.target_distance - distance_moved

        if abs(error) > 0.01:
            # Proportional control for linear velocity
            linear_speed = self.kp * abs(error)
            linear_speed = max(
                self.min_linear_speed, min(self.max_linear_speed, linear_speed)
            )
            blackboard["twist_msg"].linear.x = math.copysign(linear_speed, error)
            return CONTINUES
        elif not self.overshoot_correction:
            # Stop robot gently if within threshold
            self.overshoot_correction = True
            yasmin.YASMIN_LOG_INFO("Target distance reached, stopping.")
            return CONTINUES

        # Finalize movement
        self.start_position = None
        return ENDS
