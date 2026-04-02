# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32
# import socket
# import json

# class SensagramROSNode(Node):
#     def __init__(self, udp_ip="0.0.0.0", udp_port=5005):
#         super().__init__('sensagram_orientation_node')

#         # Create ROS2 publishers
#         self.yaw_pub = self.create_publisher(Float32, '/yaw', 10)
#         self.pitch_pub = self.create_publisher(Float32, '/pitch', 10)
#         self.roll_pub = self.create_publisher(Float32, '/roll', 10)

#         # Setup UDP socket
#         self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         self.sock.bind((udp_ip, udp_port))
#         self.sock.setblocking(False)

#         self.get_logger().info(f"Listening for Sensagram orientation UDP on {udp_ip}:{udp_port}")

#         # Timer to check UDP data at 100 Hz
#         self.create_timer(0.01, self.timer_callback)

#     def timer_callback(self):
#         try:
#             data, _ = self.sock.recvfrom(1024)
#             json_data = json.loads(data.decode())

#             values = json_data.get("values", [0.0, 0.0, 0.0])
#             # Convert to float
#             yaw = float(values[0])
#             pitch = float(values[1])
#             roll = float(values[2])

#             # Publish to ROS2 topics
#             self.yaw_pub.publish(Float32(data=yaw))
#             self.pitch_pub.publish(Float32(data=pitch))
#             self.roll_pub.publish(Float32(data=roll))

#             # Optional: print in terminal
#             self.get_logger().info(f"Yaw: {yaw:.2f}°, Pitch: {pitch:.2f}°, Roll: {roll:.2f}°")

#         except BlockingIOError:
#             pass  # No data yet
#         except json.JSONDecodeError:
#             self.get_logger().warning("Received invalid JSON")

# def main(args=None):
#     rclpy.init(args=args)
#     node = SensagramROSNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()






#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import socket
import json

class SensagramROSNode(Node):
    def __init__(self, udp_ip="0.0.0.0", udp_port=5005):
        super().__init__('sensagram_gyro_teleop')

        self.pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((udp_ip, udp_port))
        self.sock.setblocking(False)

        self.get_logger().info(f"Listening on {udp_ip}:{udp_port}")

        # Teleop tuning
        self.deadzone = 0.02

        # This defines how fast robot moves (m/s per rad/s of hand motion)
        self.linear_scale = 0.08     # Try between 0.03 – 0.15

        # Safety clamp
        self.max_linear = 0.15       # m/s

        # 30 Hz loop
        self.create_timer(1.0/30.0, self.timer_callback)

    def process(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def clamp(self, v, limit):
        return max(-limit, min(limit, v))

    def timer_callback(self):
        try:
            data, _ = self.sock.recvfrom(1024)
            json_data = json.loads(data.decode())
            values = json_data.get("values", [0.0, 0.0, 0.0])

            gx = float(values[0])
            gy = float(values[1])
            gz = float(values[2])

            # Deadzone
            gx = self.process(gx)
            gy = self.process(gy)
            gz = self.process(gz)

            # Map gyro → linear velocity (m/s)
            vx = gy * self.linear_scale
            vy = gx * self.linear_scale
            vz = gz * self.linear_scale

            # Clamp for safety
            vx = self.clamp(vx, self.max_linear)
            vy = self.clamp(vy, self.max_linear)
            vz = self.clamp(vz, self.max_linear)

            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"

            msg.twist.linear.x = vx
            msg.twist.linear.y = vy
            msg.twist.linear.z = vz

            # No rotation for now
            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = 0.0

            self.pub.publish(msg)

            self.get_logger().info(
                f"Linear m/s → x={vx:.3f}, y={vy:.3f}, z={vz:.3f}"
            )

        except BlockingIOError:
            pass
        except json.JSONDecodeError:
            self.get_logger().warning("Invalid JSON")

def main(args=None):
    rclpy.init(args=args)
    node = SensagramROSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

