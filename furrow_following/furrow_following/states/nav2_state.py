from yasmin_ros import ActionState
from yasmin.blackboard import Blackboard
from nav2_msgs.action import NavigateToPose


class Nav2State(ActionState):

    def __init__(self) -> None:
        super().__init__(
            NavigateToPose,
            "navigate_to_pose",
            self.create_nav2_goal,
        )

    def create_nav2_goal(self, blackboard: Blackboard) -> NavigateToPose.Goal:
        return blackboard["nav2_goal"]
