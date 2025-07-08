from typing import List, Tuple
from yasmin.state_machine import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL

from furrow_following.states import (
    GetGpsState,
    CheckEndFurrowState,
)
from furrow_following.states.outcomes import FURROW_CONTINUES, FURROW_ENDS


class CheckEndFurrowStateMachine(StateMachine):

    def __init__(self, poligon: List[Tuple[float, float]]) -> None:
        super().__init__([FURROW_CONTINUES, FURROW_ENDS, CANCEL])

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
                FURROW_CONTINUES: FURROW_CONTINUES,
                FURROW_ENDS: FURROW_ENDS,
            },
        )
