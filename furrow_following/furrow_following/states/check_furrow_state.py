import numpy as np
import yasmin
from yasmin import State
from yasmin.blackboard import Blackboard


class CheckFurrowState(State):

    NO_FURROW = "NO_FURROW"
    FURROW = "FURROW"

    def __init__(self) -> None:
        super().__init__([self.NO_FURROW, self.FURROW])

    def execute(self, blackboard: Blackboard) -> str:

        if self.detect_header(blackboard["processed_depth"]):
            yasmin.YASMIN_LOG_INFO("Field header detected — switching to turn mode")
            # Trigger turning logic or navigation to next row
            return self.NO_FURROW
        elif self.detect_end_of_furrow(blackboard["processed_depth"]):
            # Stop the robot or trigger crop counting wrap-up
            yasmin.YASMIN_LOG_INFO("Furrow end detected — stopping.")
            return self.NO_FURROW

        return self.FURROW

    def detect_end_of_furrow(
        self,
        depth_roi: np.ndarray,
        depth_threshold: float = 1.5,
        variance_threshold: float = 0.02,
        missing_ratio_threshold: float = 0.7,
    ) -> bool:
        """
        Returns True if the furrow has ended in this ROI.

        Criteria:
         - Median depth exceeds `depth_threshold` (furrow opens up or flattens)
         - Variance of depths is low (< variance_threshold)
         - Large portion of ROI is invalid/missing (> missing_ratio_threshold)

        Args:
            depth_roi: 2D array of depth values (meters), with invalids = np.inf or nan.
            depth_threshold: if the median depth is larger than this, we assume the furrow
                has opened up too much (i.e. ended).
            variance_threshold: if depth variation is below this, it suggests flat ground.
            missing_ratio_threshold: fraction of pixels invalid—if too many, no furrow.

        Returns:
            True if end of furrow detected, False otherwise.
        """
        # Mask out invalid depth
        valid_mask = np.isfinite(depth_roi) & (depth_roi > 0.05) & (depth_roi < 3.0)
        valid_depths = depth_roi[valid_mask]

        # If most of the ROI is invalid, assume furrow gone
        missing_ratio = 1.0 - (valid_depths.size / float(depth_roi.size))
        if missing_ratio > missing_ratio_threshold:
            yasmin.YASMIN_LOG_INFO(
                f"End of furrow: too much invalid depth ({missing_ratio:.2f})"
            )
            return True

        # If no valid pixels remain, furrow is gone
        if valid_depths.size == 0:
            yasmin.YASMIN_LOG_INFO("End of furrow: no valid depth data")
            return True

        # Check median depth
        med = np.median(valid_depths)
        if med > depth_threshold:
            yasmin.YASMIN_LOG_INFO(
                f"End of furrow: median depth {med:.2f} > threshold {depth_threshold}"
            )
            return True

        # Check variance (flatness)
        var = np.var(valid_depths)
        if var < variance_threshold:
            yasmin.YASMIN_LOG_INFO(
                f"End of furrow: depth variance {var:.4f} < threshold {variance_threshold}"
            )
            return True

        # Otherwise, furrow appears to continue
        return False

    def detect_header(
        self,
        depth_roi: np.ndarray,
        min_clear_distance: float = 1.8,
        horizontal_consistency_threshold: float = 0.1,
    ) -> bool:
        """
        Detects if the robot has reached a field header (open cross-path at row end).

        Args:
            depth_roi: 2D depth array from bottom half of image.
            min_clear_distance: minimum average depth to be considered 'open'.
            horizontal_consistency_threshold: max allowed stddev in horizontal scan to be 'open'.

        Returns:
            True if a header is detected.
        """
        valid_mask = np.isfinite(depth_roi) & (depth_roi > 0.05) & (depth_roi < 5.0)
        if not np.any(valid_mask):
            self.get_logger().info("Header detection: all depth invalid")
            return False  # Can't conclude anything

        valid_depth = depth_roi[valid_mask]
        mean_depth = np.mean(valid_depth)
        std_horizontal = np.std(np.mean(depth_roi, axis=0))  # Variation across columns

        if (
            mean_depth > min_clear_distance
            and std_horizontal < horizontal_consistency_threshold
        ):
            self.get_logger().info(
                f"Header detected — mean depth: {mean_depth:.2f}, horizontal std: {std_horizontal:.2f}"
            )
            return True

        return False
