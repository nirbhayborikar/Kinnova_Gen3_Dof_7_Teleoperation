#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# Safe import handling (IMPORTANT for Docker)
try:
    from apriltag_msgs.msg import AprilTagDetectionArray
except ImportError:
    AprilTagDetectionArray = None


class WebcamAprilTagNode(Node):

    def __init__(self):
        super().__init__('webcam_apriltag_node')

        if AprilTagDetectionArray is None:
            self.get_logger().error(
                "apriltag_msgs is NOT available in this environment!"
            )
            self.get_logger().error(
                "Make sure you sourced /opt/ros/jazzy/setup.bash and workspace setup.bash"
            )
            return

        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',   # or /tag_detections depending on your launch
            self.callback,
            10
        )

        self.get_logger().info("AprilTag listener node started...")

    def callback(self, msg):

        if not msg.detections:
            self.get_logger().info("No AprilTag detected")
            return

        self.get_logger().info(f"Detected {len(msg.detections)} tag(s)")

        for d in msg.detections:
            self.get_logger().info(
                f"ID: {d.id} | "
                f"Center: ({d.centre.x:.2f}, {d.centre.y:.2f}) | "
                f"Margin: {d.decision_margin:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)

    node = WebcamAprilTagNode()

    if AprilTagDetectionArray is not None:
        rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()