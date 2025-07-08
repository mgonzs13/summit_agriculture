#!/usr/bin/env python3

import rclpy

import yasmin
from yasmin.blackboard import Blackboard
from yasmin_ros.ros_logs import set_ros_loggers
from yasmin_viewer.yasmin_viewer_pub import YasminViewerPub

from furrow_following.state_machines import FurrowFollowingStateMachine


def main() -> None:
    # Configure YASMIN to use ROS-based logging
    rclpy.init()
    set_ros_loggers()

    # Create the state machine
    sm = FurrowFollowingStateMachine()

    # Ensure the state machine cancels on shutdown
    def on_shutdown():
        if sm.is_running():
            sm.cancel_state()

    rclpy.get_default_context().on_shutdown(on_shutdown)

    # Launch YASMIN Viewer publisher for state visualization
    YasminViewerPub("FURROW_FOLLOWING", sm)

    # Initialize blackboard with counter values
    blackboard = Blackboard()

    # Run the state machine and log the outcome
    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(outcome)
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(str(e))

    # Shutdown ROS
    rclpy.shutdown()


if __name__ == "__main__":
    main()
