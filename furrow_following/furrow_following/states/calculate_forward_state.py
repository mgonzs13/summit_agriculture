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

    def euclidean_distance(self, x1, y1, x2, y2) -> float:
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def execute(self, blackboard: Blackboard) -> str:
        msg: Odometry = blackboard["odom_msg"]
        position = msg.pose.pose.position

        blackboard["twist_msg"] = Twist()

        # Store initial position
        if "start_position" not in blackboard or blackboard["start_position"] is None:
            blackboard["start_position"] = (position.x, position.y)
            yasmin.YASMIN_LOG_INFO(
                f"Start position: x={position.x:.2f}, y={position.y:.2f}"
            )
            return CONTINUES

        start_x, start_y = blackboard["start_position"]
        distance_moved = self.euclidean_distance(start_x, start_y, position.x, position.y)
        yasmin.YASMIN_LOG_INFO(f"Moved: {distance_moved:.2f} m")

        if distance_moved < self.target_distance:
            blackboard["twist_msg"].linear.x = 0.2  # m/s
            return CONTINUES

        return ENDS
