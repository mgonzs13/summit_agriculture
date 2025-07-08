import numpy as np
import yasmin
from yasmin import State
from yasmin.blackboard import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from geometry_msgs.msg import Twist


class CalculateTwistState(State):

    def __init__(
        self, Kp: float = 0.015, linear_speed: float = 0.4, max_angular: float = 0.4
    ) -> None:
        super().__init__([SUCCEED, ABORT])

        self.Kp = Kp
        self.linear_speed = linear_speed
        self.max_angular = max_angular
        self.highest_point_memory = None

    def execute(self, blackboard: Blackboard) -> str:

        processed_depth: np.ndarray = blackboard["processed_depth"]
        depth_image: np.ndarray = blackboard["depth_image"]

        if processed_depth is None or depth_image is None:
            yasmin.YASMIN_LOG_WARN("Missing required depth data on blackboard")
            return ABORT

        h, w = depth_image.shape

        # Focus on center third of the image (ROI)
        center_start = w // 3
        center_end = 2 * w // 3
        center_region = processed_depth[:, center_start:center_end]

        # Mask valid depth values
        valid_mask = (
            np.isfinite(center_region) & (center_region > 0.1) & (center_region < 2.0)
        )

        if not np.any(valid_mask):
            yasmin.YASMIN_LOG_WARN("No valid depth values in center region")
            return ABORT

        # Find the minimum depth (i.e., closest/highest point in furrow)
        min_depth_value = np.min(center_region[valid_mask])
        min_positions = np.where(center_region == min_depth_value)

        if len(min_positions[0]) == 0:
            yasmin.YASMIN_LOG_WARN(
                "Valid mask passed but no minimum depth position found"
            )
            return ABORT

        # Use first detected minimum depth location
        highest_row = min_positions[0][0]
        highest_col_center = min_positions[1][0]
        highest_col = center_start + highest_col_center

        yasmin.YASMIN_LOG_INFO(
            f"Highest point at col: {highest_col}, depth: {min_depth_value:.3f} m"
        )

        # Temporal smoothing of column position
        if self.highest_point_memory is None:
            self.highest_point_memory = highest_col
        else:
            self.highest_point_memory = (
                0.5 * self.highest_point_memory + 0.5 * highest_col
            )

        target_center = self.highest_point_memory

        # === CONTROL ===
        # Error = horizontal distance from image center
        error = (w / 2) - target_center

        twist = Twist()
        twist.linear.x = self.linear_speed

        # Apply bounded proportional control to angular velocity
        twist.angular.z = np.clip(self.Kp * error, -self.max_angular, self.max_angular)

        # Push twist to blackboard
        blackboard["twist_msg"] = twist

        yasmin.YASMIN_LOG_INFO(
            f"Calculated Twist: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}, error={error:.1f}"
        )

        return SUCCEED
