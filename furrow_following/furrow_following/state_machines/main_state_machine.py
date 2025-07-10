from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL

from furrow_following.state_machines.check_end_furrow_state_machine import (
    CheckEndFurrowStateMachine,
)
from furrow_following.state_machines.furrow_following_state_machine import (
    FurrowFollowingStateMachine,
)
from furrow_following.state_machines.turning_state_machine import TurningStateMachine
from furrow_following.state_machines.moving_forward_state_machine import (
    MovingForwardStateMachine,
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
                    (42.61280378018076, -5.5655960952234915),
                    (42.61280378018076, -5.565724812262146),
                    (42.61289831883529, -5.565724812262146),
                ]
            ),
            {
                ENDS: "TURNING_1",
                CONTINUES: "FURROW_FOLLOWING",
            },
        )

        self.add_state(
            "TURNING_1",
            TurningStateMachine(),
            {
                ENDS: "MOVING_FORWARD",
                CANCEL: CANCEL,
            },
        )

        self.add_state(
            "MOVING_FORWARD",
            MovingForwardStateMachine(),
            {
                ENDS: "TURNING_2",
                CANCEL: CANCEL,
            },
        )

        self.add_state(
            "TURNING_2",
            TurningStateMachine(),
            {
                ENDS: "FURROW_FOLLOWING",
                CANCEL: CANCEL,
            },
        )
