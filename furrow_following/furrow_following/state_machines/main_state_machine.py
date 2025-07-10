from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL

from furrow_following.state_machines.check_end_furrow_state_machine import (
    CheckEndFurrowStateMachine,
)
from furrow_following.state_machines.furrow_following_state_machine import (
    FurrowFollowingStateMachine,
)
from furrow_following.state_machines.move_to_next_furrow_state_machine import (
    MoveToNextFurrowStateMachine,
)
from furrow_following.states.outcomes import ENDS, CONTINUES


class MainStateMachine(StateMachine):

    def __init__(self) -> None:
        super().__init__([SUCCEED, ABORT, CANCEL])

        self.add_state(
            "FURROW_FOLLOWING",
            FurrowFollowingStateMachine(),
            {
                SUCCEED: "CHECKING_END_FURROW",
                ABORT: ABORT,
                CANCEL: CANCEL,
            },
        )

        self.add_state(
            "CHECKING_END_FURROW",
            CheckEndFurrowStateMachine(
                [
                    (42.61289831883529, -5.5655960952234915),
                    (42.61280368018076, -5.5655960952234915),
                    (42.61280368018076, -5.565724812262146),
                    (42.61289831883529, -5.565724812262146),
                ]
            ),
            {
                ENDS: "MOVING_TO_NEXT_FURROW",
                CONTINUES: "FURROW_FOLLOWING",
            },
        )

        self.add_state(
            "MOVING_TO_NEXT_FURROW",
            MoveToNextFurrowStateMachine(),
            {
                SUCCEED: "FURROW_FOLLOWING",
                CANCEL: CANCEL,
            },
        )
