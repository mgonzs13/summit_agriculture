from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL


from furrow_following.state_machines.turning_state_machine import TurningStateMachine
from furrow_following.state_machines.moving_forward_state_machine import (
    MovingForwardStateMachine,
)
from furrow_following.states.outcomes import ENDS


class MoveToNextFurrowStateMachine(StateMachine):

    def __init__(self) -> None:
        super().__init__([SUCCEED, CANCEL])

        self.add_state(
            "MOVING_FORWARD_1",
            MovingForwardStateMachine(0.5),
            {
                ENDS: "TURNING_1",
                CANCEL: CANCEL,
            },
        )

        self.add_state(
            "TURNING_1",
            TurningStateMachine(-80.0),
            {
                ENDS: "MOVING_FORWARD_2",
                CANCEL: CANCEL,
            },
        )

        self.add_state(
            "MOVING_FORWARD_2",
            MovingForwardStateMachine(0.5),
            {
                ENDS: "TURNING_2",
                CANCEL: CANCEL,
            },
        )

        self.add_state(
            "TURNING_2",
            TurningStateMachine(-80.0),
            {
                ENDS: "MOVING_FORWARD_3",
                CANCEL: CANCEL,
            },
        )

        self.add_state(
            "MOVING_FORWARD_3",
            MovingForwardStateMachine(1.0),
            {
                ENDS: SUCCEED,
                CANCEL: CANCEL,
            },
        )
