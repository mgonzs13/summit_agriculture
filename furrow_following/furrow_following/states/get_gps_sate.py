from yasmin.blackboard import Blackboard
from yasmin_ros import MonitorState
from yasmin_ros.basic_outcomes import SUCCEED
from sensor_msgs.msg import NavSatFix


class GetGpsState(MonitorState):

    def __init__(self, topic_name: str) -> None:
        super().__init__(
            NavSatFix,
            topic_name,
            [SUCCEED],
            monitor_handler=self.handle_gps,
        )

    def handle_gps(self, blackboard: Blackboard, msg: NavSatFix) -> str:
        blackboard["gps_msg"] = msg
        return SUCCEED
