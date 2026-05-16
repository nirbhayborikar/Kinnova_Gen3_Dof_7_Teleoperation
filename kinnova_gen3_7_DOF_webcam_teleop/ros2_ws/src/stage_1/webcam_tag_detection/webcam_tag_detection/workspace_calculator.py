#!/usr/bin/env python3
"""
Workspace Mapper for Kinova Gen3.
Prints the real-time X, Y, Z coordinates of the end-effector to help you find your min/max limits.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException

class WorkspaceMapper(Node):
    def __init__(self):
        super().__init__('workspace_mapper')

        # --- Parameters ---
        # Change 'end_effector_link' to 'tool_frame' or 'grasping_frame' if needed
        self.base_frame = 'base_link'
        self.tool_frame = 'grasping_frame' 

        # --- TF2 Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Print the coordinates twice a second (2.0 Hz) so it's easy to read
        self.timer = self.create_timer(0.5, self._timer_cb)

        self.get_logger().info("=========================================")
        self.get_logger().info("  Workspace Mapper Active!")
        self.get_logger().info(" Move the robot arm to find your limits.")
        self.get_logger().info("=========================================")

    def _timer_cb(self):
        try:
            # Ask ROS exactly where the tool is relative to the heavy base
            t = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tool_frame,
                rclpy.time.Time()
            )

            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z

            # Print it cleanly to the terminal
            self.get_logger().info(f" CURRENT POSITION -> X: {x:+.3f} | Y: {y:+.3f} | Z: {z:+.3f}")

        except TransformException as e:
            self.get_logger().warning(f"Waiting for TF data... ({e})", throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()