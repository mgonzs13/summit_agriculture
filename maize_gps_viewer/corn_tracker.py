#!/usr/bin/env python3

import csv
import math
import os
import cv2
import rclpy
import shutil
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from yolo_msgs.msg import DetectionArray

CONFIDENCE_THRESHOLD = 0.6
EARTH_RADIUS = 6378137.0
CSV_FILE = "corn_detections.csv"


def enu_to_wgs84(lat_origin, lon_origin, dx, dy):
    lat_rad = math.radians(lat_origin)
    lat = lat_rad + dy / EARTH_RADIUS
    lon = math.radians(lon_origin) + dx / (EARTH_RADIUS * math.cos(lat_rad))
    return math.degrees(lat), math.degrees(lon)


class CornTracker(Node):
    def __init__(self):
        super().__init__("corn_tracker")
        self.gps_data = None
        self.detected_ids = set()
        self.bridge = CvBridge()
        self.last_image = None

        if os.path.isdir("images"):
            shutil.rmtree("images")

        os.makedirs("images", exist_ok=True)

        self.create_subscription(NavSatFix, "/robot/gps/fix", self.gps_callback, 10)
        self.create_subscription(DetectionArray, "/yolo/detections_3d", self.detection_callback, 10)
        self.create_subscription(Image, "/robot/zed2/zed_node/rgb/image_rect_color", self.image_callback, 10)

        self.create_csv()

    def create_csv(self):
        with open(CSV_FILE, "w", newline="") as file:
            csv.writer(file).writerow(["id", "latitude", "longitude", "image", "confidence"])

    def append_csv(self, corn_id, lat, lon, image_name, confidence):
        with open(CSV_FILE, "a", newline="") as f:
            csv.writer(f).writerow([corn_id, f"{lat:.10f}", f"{lon:.10f}", image_name, f"{confidence:.5f}"])

    def gps_callback(self, msg):
        self.gps_data = msg

    def image_callback(self, msg):
        self.last_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

    def detection_callback(self, msg):
        if self.gps_data is None or self.last_image is None:
            return

        lat_origin, lon_origin = self.gps_data.latitude, self.gps_data.longitude

        for detection in msg.detections:
            if detection.class_name != "corn" or detection.score < CONFIDENCE_THRESHOLD:
                continue

            corn_id = detection.id
            if corn_id in self.detected_ids:
                continue
            self.detected_ids.add(corn_id)

            position = detection.bbox3d.center.position
            lat, lon = enu_to_wgs84(lat_origin, lon_origin, position.x, position.y)

            bbox = detection.bbox
            cx, cy = int(bbox.center.position.x), int(bbox.center.position.y)
            width, height = int(bbox.size.x), int(bbox.size.y)
            x0, y0 = max(cx - width // 2, 0), max(cy - height // 2, 0)
            x1, y1 = (
                min(cx + width // 2, self.last_image.shape[1] - 1),
                min(cy + height // 2, self.last_image.shape[0] - 1),
            )

            crop = self.last_image[y0:y1, x0:x1]
            image_name = f"{corn_id}.png"
            cv2.imwrite(os.path.join("images", image_name), crop)

            self.append_csv(corn_id, lat, lon, image_name, detection.score)

            total = len(self.detected_ids)
            self.get_logger().info(f"{corn_id}: ENU COORDS ({position.x:.10f}, {position.y:.10f}) | "f"GPS COORDS ({lat:.10f}, {lon:.10f}) | CONFIDENCE {detection.score:.5f} | "f"TOTAL CORNS {total}")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CornTracker())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
