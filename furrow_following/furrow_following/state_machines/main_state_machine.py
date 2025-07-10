from yasmin import StateMachine, Concurrence
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
from furrow_following.states.outcomes import ENDS


class MainStateMachine(StateMachine):

    def __init__(self) -> None:
        super().__init__([SUCCEED, ABORT, CANCEL])

        self.furrow_following_sm = FurrowFollowingStateMachine()
        self.check_end_furrow_sm = CheckEndFurrowStateMachine(
            [
                (42.61289831883529, -5.5655960952234915),
                (42.61280378018076, -5.5655960952234915),
                (42.61280378018076, -5.565724812262146),
                (42.61289831883529, -5.565724812262146),
            ]
        )

        self.add_state(
            "FURROW_FOLLOWING",
            Concurrence(
                [self.furrow_following_sm, self.check_end_furrow_sm],
                default_outcome=SUCCEED,
                outcome_map={
                    SUCCEED: {
                        self.check_end_furrow_sm: ENDS,
                        self.furrow_following_sm: SUCCEED,
                    },
                    CANCEL: {self.check_end_furrow_sm: CANCEL},
                    CANCEL: {self.furrow_following_sm: CANCEL},
                    ABORT: {self.furrow_following_sm: ABORT},
                },
            ),
            {
                SUCCEED: "TURNING_1",
                ABORT: ABORT,
                CANCEL: CANCEL,
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
