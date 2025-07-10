from yasmin.blackboard import Blackboard
from yasmin_ros import MonitorState
from yasmin_ros.basic_outcomes import SUCCEED
from nav_msgs.msg import Odometry


class GetOdomState(MonitorState):

    def __init__(self, topic_name: str) -> None:
        super().__init__(
            Odometry,
            topic_name,
            [SUCCEED],
            monitor_handler=self.handle_gps,
        )

    def handle_gps(self, blackboard: Blackboard, msg: Odometry) -> str:
        blackboard["odom_msg"] = msg
        return SUCCEED
