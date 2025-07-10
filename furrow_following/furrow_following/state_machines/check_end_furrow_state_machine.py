from typing import List, Tuple
from yasmin.state_machine import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL

from furrow_following.states import (
    GetGpsState,
    CheckEndFurrowState,
)
from furrow_following.states.outcomes import CONTINUES, ENDS


class CheckEndFurrowStateMachine(StateMachine):

    def __init__(self, poligon: List[Tuple[float, float]]) -> None:
        super().__init__([ENDS, CANCEL])

        self.add_state(
            "GETTING_GPS",
            GetGpsState("/robot/gps/fix"),
            {
                SUCCEED: "CHECKING_END_FURROW",
                CANCEL: CANCEL,
            },
        )

        self.add_state(
            "CHECKING_END_FURROW",
            CheckEndFurrowState(poligon),
            {
                CONTINUES: "GETTING_GPS",
                ENDS: ENDS,
            },
        )
