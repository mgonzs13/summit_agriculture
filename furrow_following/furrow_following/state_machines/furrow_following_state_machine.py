from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from furrow_following.states.outcomes import ENDS, CONTINUES
from furrow_following.states import (
    GetDepthImageState,
    CalculateTwistState,
    DriveState,
    DepthCheckEndFurrowState,
)


class FurrowFollowingStateMachine(StateMachine):

    def __init__(self) -> None:
        super().__init__([SUCCEED, ABORT, CANCEL, ENDS])

        self.add_state(
            "GETTING_DEPTH_IMAGE",
            GetDepthImageState("/depth_centered"),
            {
                SUCCEED: "CHECKING_END_FURROW",
                CANCEL: CANCEL,
            },
        )

        self.add_state(
            "CHECKING_END_FURROW",
            DepthCheckEndFurrowState(),
            {
                ENDS: ENDS,
                CONTINUES: "CALCULATING_TWIST",
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
                SUCCEED: SUCCEED,
            },
        )
