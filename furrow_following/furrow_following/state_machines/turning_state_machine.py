from yasmin.state_machine import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL

from furrow_following.states import GetOdomState, CalculateTurnState, DriveState
from furrow_following.states.outcomes import CONTINUES, ENDS


class TurningStateMachine(StateMachine):

    def __init__(self, target_angle: float = -90.0) -> None:
        super().__init__([ENDS, CANCEL])

        self.add_state(
            "GETTING_ODOM",
            GetOdomState("/global_odom"),
            {
                SUCCEED: "CALCULATING_TURN",
                CANCEL: CANCEL,
            },
        )

        self.add_state(
            "CALCULATING_TURN",
            CalculateTurnState(target_angle),
            {
                CONTINUES: "DRIVING",
                ENDS: ENDS,
            },
        )

        self.add_state(
            "DRIVING",
            DriveState("/robot/robotnik_base_control/cmd_vel"),
            {
                SUCCEED: "GETTING_ODOM",
            },
        )
