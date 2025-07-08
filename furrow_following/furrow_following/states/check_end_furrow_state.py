from typing import List, Tuple
import yasmin
from yasmin import State
from yasmin.blackboard import Blackboard
from sensor_msgs.msg import NavSatFix

from furrow_following.states.outcomes import FURROW_CONTINUES, FURROW_ENDS


class CheckEndFurrowState(State):

    def __init__(self, poligon: List[Tuple[float, float]]) -> None:
        super().__init__([FURROW_CONTINUES, FURROW_ENDS])
        self.poligon = poligon

    def execute(self, blackboard: Blackboard) -> str:

        msg: NavSatFix = blackboard["gps_msg"]

        if self.is_point_in_polygon(msg.latitude, msg.longitude, self.poligon):
            yasmin.YASMIN_LOG_INFO("End of furrow")
            return FURROW_CONTINUES

        return FURROW_ENDS

    def is_point_in_polygon(
        self, x: float, y: float, polygon: List[Tuple[float, float]]
    ) -> bool:
        """
        Determine if a point is inside a polygon using the ray casting algorithm.

        Parameters:
            x, y (float): The coordinates of the point.
            polygon (list of tuples): A list of (x, y) tuples representing the polygon vertices.

        Returns:
            bool: True if the point is inside the polygon, False otherwise.
        """
        num = len(polygon)
        inside = False

        px, py = x, y
        for i in range(num):
            j = (i - 1) % num
            xi, yi = polygon[i]
            xj, yj = polygon[j]

            intersect = ((yi > py) != (yj > py)) and (
                px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi
            )
            if intersect:
                inside = not inside

        return inside
