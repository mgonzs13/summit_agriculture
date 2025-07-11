from typing import List, Tuple
from yasmin.state_machine import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, TIMEOUT

from furrow_following.states import (
    GetGpsState,
    GpsCheckEndFurrowState,
)
from furrow_following.states.outcomes import CONTINUES, ENDS


class GpsCheckEndFurrowStateMachine(StateMachine):

    def __init__(self, poligon: List[Tuple[float, float]]) -> None:
        super().__init__([ENDS, CONTINUES, CANCEL])

        self.add_state(
            "GETTING_GPS",
            GetGpsState("/robot/gps/fix"),
            {
                SUCCEED: "CHECKING_END_FURROW",
                CANCEL: CANCEL,
                TIMEOUT: CONTINUES,
            },
        )

        self.add_state(
            "CHECKING_END_FURROW",
            GpsCheckEndFurrowState(poligon),
            {
                CONTINUES: CONTINUES,
                ENDS: ENDS,
            },
        )
