import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time


class JointMonitor(Node):

    def __init__(self):
        super().__init__('joint_monitor')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.callback,
            10
        )

        self.prev_positions = None
        self.prev_time = None

        # thresholds (tune these!)
        self.position_threshold = 0.01   # radians
        self.velocity_threshold = 0.5    # rad/s

    def callback(self, msg: JointState):
        current_positions = np.array(msg.position)
        current_time = time.time()

        if self.prev_positions is None:
            self.prev_positions = current_positions
            self.prev_time = current_time
            return

        dt = current_time - self.prev_time
        if dt == 0:
            return

        # compute changes
        delta = np.abs(current_positions - self.prev_positions)
        velocity = delta / dt

        max_delta = np.max(delta)
        max_velocity = np.max(velocity)

        # condition: movement detected
        if max_delta > self.position_threshold or max_velocity > self.velocity_threshold:
            self.get_logger().info(
                f"⚡ Movement detected | Δ={max_delta:.4f}, vel={max_velocity:.4f}"
            )
        else:
            self.get_logger().info("Joint angles not changing")

        # update previous
        self.prev_positions = current_positions
        self.prev_time = current_time


def main():
    rclpy.init()
    node = JointMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





# python3 joint_tracker.py
# apt-get update && apt-get install -y \
#     python3-jinja2 \
#     python3-typeguard