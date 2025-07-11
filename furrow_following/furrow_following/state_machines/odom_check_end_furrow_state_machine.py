from yasmin.state_machine import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL

from furrow_following.states import (
    GetOdomState,
    OdomCheckEndFurrowState,
)
from furrow_following.states.outcomes import CONTINUES, ENDS


class OdomCheckEndFurrowStateMachine(StateMachine):

    def __init__(self, furrow_length: float) -> None:
        super().__init__([ENDS, CONTINUES, CANCEL])

        self.add_state(
            "GETTING_ODOM",
            GetOdomState("/global_odom"),
            {
                SUCCEED: "CHECKING_END_FURROW",
                CANCEL: CANCEL,
            },
        )
        self.add_state(
            "CHECKING_END_FURROW",
            OdomCheckEndFurrowState(furrow_length),
            {
                CONTINUES: CONTINUES,
                ENDS: ENDS,
            },
        )
