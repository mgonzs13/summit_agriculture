from yasmin.state_machine import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL

from furrow_following.states import (
    GetDepthImageState,
    CalculateTwistState,
    DriveState,
)
from furrow_following.state_machines.check_end_furrow_state_machine import (
    CheckEndFurrowStateMachine,
)
from furrow_following.states.outcomes import FURROW_CONTINUES, FURROW_ENDS


class FurrowFollowingStateMachine(StateMachine):

    def __init__(self) -> None:
        super().__init__([SUCCEED, ABORT, CANCEL])

        self.add_state(
            "CHECKING_END_FURROW",
            CheckEndFurrowStateMachine(
                [
                    (42.61290, -5.56559),
                    (42.61290, -5.56573),
                    (42.61280, -5.56559),
                    (42.61280, -5.56573),
                ]
            ),
            {
                FURROW_ENDS: SUCCEED,
                FURROW_CONTINUES: "GETTING_DEPTH_IMAGE",
            },
        )

        self.add_state(
            "GETTING_DEPTH_IMAGE",
            GetDepthImageState("/depth_centered"),
            {
                SUCCEED: "CALCULATING_TWIST",
                CANCEL: CANCEL,
            },
        )

        self.add_state(
            "CALCULATING_TWIST",
            CalculateTwistState(),
            {
                SUCCEED: "DRIVING",
                ABORT: ABORT,
            },
        )

        self.add_state(
            "DRIVING",
            DriveState("/robot/robotnik_base_control/cmd_vel"),
            {
                SUCCEED: "GETTING_DEPTH_IMAGE",
            },
        )
