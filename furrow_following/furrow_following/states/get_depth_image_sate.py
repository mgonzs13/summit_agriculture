import numpy as np
import yasmin
from yasmin.blackboard import Blackboard
from yasmin_ros import MonitorState
from yasmin_ros.basic_outcomes import SUCCEED
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class GetDepthImageState(MonitorState):

    def __init__(self, topic_name: str) -> None:
        super().__init__(
            Image,
            topic_name,
            [SUCCEED],
            monitor_handler=self.handle_image,
        )
        self.bridge = CvBridge()

    def handle_image(self, blackboard: Blackboard, msg: Image) -> str:
        try:
            # Convert to OpenCV format
            depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Image conversion failed: {e}")
            return

        # Focus on lower half (assumed furrow region)
        h, w = depth_image.shape
        roi_depth = depth_image[h // 2 :, :]

        # Pre-process depth image - remove invalid values and limit range
        valid_mask = np.isfinite(roi_depth) & (roi_depth > 0.05) & (roi_depth < 3.0)
        processed_depth = np.copy(roi_depth)
        processed_depth[~valid_mask] = (
            np.inf
        )  # Set invalid areas to infinity to ignore them

        blackboard["processed_depth"] = processed_depth
        blackboard["depth_image"] = depth_image
        return SUCCEED
