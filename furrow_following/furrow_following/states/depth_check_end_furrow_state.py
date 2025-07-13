import numpy as np
from scipy.ndimage import uniform_filter1d

import yasmin
from yasmin import State
from yasmin.blackboard import Blackboard

from furrow_following.states.outcomes import CONTINUES, ENDS


class DepthCheckEndFurrowState(State):

    def __init__(self) -> None:
        super().__init__([CONTINUES, ENDS])
        self.depth_history = []

    def execute(self, blackboard: Blackboard) -> str:

        if self.detect_end_of_furrow_with_headland(blackboard["depth_image"]):
            self.depth_history.clear()
            yasmin.YASMIN_LOG_INFO("End of furrow")
            return ENDS

        return CONTINUES

    def detect_end_of_furrow_with_headland(
        self,
        depth_image: np.ndarray,
        scan_row_ratio: float = 0.85,
        min_valid_points: int = 40,
        flatness_std_threshold: float = 0.01,
        depth_rise_threshold: float = 0.12,
        history_len: int = 30,
    ) -> bool:
        """
        Detect the end of a furrow considering headlands using depth image analysis.

        Parameters:
            depth_image (np.ndarray): Raw or filtered depth image (in meters).
            scan_row_ratio (float): Relative vertical position (0 = top, 1 = bottom) for scanline.
            min_valid_points (int): Minimum valid depth values to accept analysis.
            flatness_std_threshold (float): If std of scanline is below this, assume headland.
            depth_rise_threshold (float): Rising depth amount (meters) indicating terrain elevation.
            history_len (int): Number of frames to track increasing depth.

        Returns:
            bool: True if end of furrow (i.e., headland) is detected.
        """

        h, w = depth_image.shape
        scan_row = int(h * scan_row_ratio)
        scanline = depth_image[scan_row, :]
        valid_mask = (scanline > 0.1) & (scanline < 3.0)
        valid_depths = scanline[valid_mask]

        if valid_depths.size < min_valid_points:
            # Not enough information — possibly occlusion or end
            yasmin.YASMIN_LOG_INFO(
                f"Not enough valid points in scan row {scan_row}: {valid_depths.size} < {min_valid_points}"
            )
            return True

        # Flatness check (headland is typically flat terrain)
        std_dev = np.std(valid_depths)
        if std_dev < flatness_std_threshold:
            yasmin.YASMIN_LOG_INFO(
                f"Flat terrain detected at row {scan_row} with std {std_dev:.3f} m"
            )
            return True

        # Minimum depth value (bottom of the furrow)
        current_min_depth = np.min(valid_depths)

        self.depth_history.append(current_min_depth)

        # Keep recent depth history
        if len(self.depth_history) > history_len:
            self.depth_history.pop(0)

        # Check for rising depth trend
        if len(self.depth_history) == history_len:
            smoothed = uniform_filter1d(self.depth_history, size=3)
            depth_deltas = np.diff(smoothed)
            rising = np.all(depth_deltas > depth_rise_threshold)
            if rising:
                yasmin.YASMIN_LOG_INFO(
                    f"Rising depth trend detected: {depth_deltas} > {depth_rise_threshold}"
                )
                return True

        return False
