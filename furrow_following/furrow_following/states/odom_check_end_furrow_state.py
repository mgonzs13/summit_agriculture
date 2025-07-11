import math
import yasmin
from yasmin import State
from yasmin.blackboard import Blackboard
from nav_msgs.msg import Odometry

from furrow_following.states.outcomes import CONTINUES, ENDS


class OdomCheckEndFurrowState(State):

    def __init__(self, furrow_length: float) -> None:
        super().__init__([CONTINUES, ENDS])
        self.furrow_length = furrow_length
        self.start_position = None

    def euclidean_distance(self, x1, y1, x2, y2) -> float:
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def execute(self, blackboard: Blackboard) -> str:
        msg: Odometry = blackboard["odom_msg"]
        position = msg.pose.pose.position

        # Store initial position
        if self.start_position is None:
            self.start_position = (position.x, position.y)
            yasmin.YASMIN_LOG_INFO(
                f"Start position: x={position.x:.2f}, y={position.y:.2f}"
            )
            return CONTINUES

        start_x, start_y = self.start_position
        distance_moved = self.euclidean_distance(start_x, start_y, position.x, position.y)
        yasmin.YASMIN_LOG_INFO(f"Moved: {distance_moved:.2f} m")

        if distance_moved < self.furrow_length:
            return CONTINUES

        self.start_position = None
        return ENDS
