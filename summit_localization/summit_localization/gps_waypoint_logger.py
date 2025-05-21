#!/usr/bin/env python3

import os
import sys
import yaml
import tkinter as tk
from tkinter import messagebox

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32  # Importar tipo correcto para el heading

class GpsGuiLogger(tk.Tk, Node):
    """
    ROS2 node to log GPS waypoints to a file
    """

    def __init__(self, logging_file_path):
        tk.Tk.__init__(self)
        Node.__init__(self, "gps_waypoint_logger")
        self.title("GPS Waypoint Logger")

        self.logging_file_path = logging_file_path

        self.gps_pose_label = tk.Label(self, text="Current Coordinates:")
        self.gps_pose_label.pack()
        self.gps_pose_textbox = tk.Label(self, text="", width=60)
        self.gps_pose_textbox.pack()

        self.log_gps_wp_button = tk.Button(
            self, text="Log GPS Waypoint", command=self.log_waypoint
        )
        self.log_gps_wp_button.pack()

        self.gps_subscription = self.create_subscription(
            NavSatFix, "/robot/gps/fix", self.gps_callback, 1
        )
        self.last_gps_position = NavSatFix()

        self.imu_subscription = self.create_subscription(
            Float32, "/imu/compass_heading", self.imu_callback, 1
        )
        self.last_heading = 0.0

    def gps_callback(self, msg: NavSatFix):
        """
        Callback to store the last GPS pose
        """
        self.last_gps_position = msg
        self.updateTextBox()

    def imu_callback(self, msg: Float32):
        """
        Callback to store the last heading from Float32 message
        """
        self.last_heading = msg.data
        self.updateTextBox()

    def updateTextBox(self):
        """
        Function to update the GUI with the last coordinates
        """
        self.gps_pose_textbox.config(
            text=f"Lat: {self.last_gps_position.latitude:.6f}, Lon: {self.last_gps_position.longitude:.6f}, Yaw: {self.last_heading:.2f} rad"
        )

    def log_waypoint(self):
        """
        Function to save a new waypoint to a file
        """
        try:
            with open(self.logging_file_path, "r") as yaml_file:
                existing_data = yaml.safe_load(yaml_file)
        except FileNotFoundError:
            existing_data = {"waypoints": []}
        except Exception as ex:
            messagebox.showerror("Error", f"Error logging position: {str(ex)}")
            return

        data = {
            "latitude": self.last_gps_position.latitude,
            "longitude": self.last_gps_position.longitude,
            "yaw": self.last_heading,
        }
        existing_data["waypoints"].append(data)

        try:
            with open(self.logging_file_path, "w") as yaml_file:
                yaml.dump(existing_data, yaml_file, default_flow_style=False)
        except Exception as ex:
            messagebox.showerror("Error", f"Error logging position: {str(ex)}")
            return

        messagebox.showinfo("Info", "Waypoint logged successfully")


def main(args=None):
    rclpy.init(args=args)

    default_yaml_file_path = os.path.expanduser("~/gps_waypoints.yaml")
    yaml_file_path = sys.argv[1] if len(sys.argv) > 1 else default_yaml_file_path

    gps_gui_logger = GpsGuiLogger(yaml_file_path)

    while rclpy.ok():
        rclpy.spin_once(gps_gui_logger, timeout_sec=0.1)
        gps_gui_logger.update()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
