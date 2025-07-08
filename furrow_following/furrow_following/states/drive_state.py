from yasmin.blackboard import Blackboard
from yasmin_ros import PublisherState
from geometry_msgs.msg import Twist


class DriveState(PublisherState):

    def __init__(self, cmd_vel_topic: str):
        super().__init__(Twist, cmd_vel_topic, self.get_twist_msg)

    def get_twist_msg(self, blackboard: Blackboard) -> Twist:
        return blackboard["twist_msg"]
