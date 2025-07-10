from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT
from furrow_following.states import GetOdomState, CalculateNextFurrowState, Nav2State


class MoveToNextFurrowStateMachine(StateMachine):

    def __init__(self) -> None:
        super().__init__([SUCCEED, ABORT, CANCEL])

        self.add_state(
            "GETTING_ODOM",
            GetOdomState("/global_odom"),
            {
                SUCCEED: "CALCULATING_NEXT_FURROW",
                CANCEL: CANCEL,
            },
        )

        self.add_state(
            "CALCULATING_NEXT_FURROW",
            CalculateNextFurrowState(0.5),
            {
                SUCCEED: "NAVIGATING",
            },
        )

        self.add_state(
            "NAVIGATING",
            Nav2State(),
            {
                SUCCEED: SUCCEED,
                ABORT: ABORT,
                CANCEL: CANCEL,
            },
        )
