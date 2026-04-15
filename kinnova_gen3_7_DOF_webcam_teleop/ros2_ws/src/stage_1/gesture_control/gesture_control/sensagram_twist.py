#!/usr/bin/env python3
"""
UDP IMU Teleop Node for Kinova Gen3 7-DOF.
Listens for Phone Orientation Data (Quaternion) via Sensagram JSON over WiFi.

Mappings:
  - Phone Pitch (Tilt Forward/Back) = Robot Z (Up/Down)
  - Phone Roll  (Tilt Left/Right)   = Robot X (Forward/Backward)
  - Phone Yaw   (Rotate Left/Right) = Robot Y (Left/Right)
  - Phone Flat = Zero Velocity
"""

import socket
import math
import json
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


class UdpTwistTeleopNode(Node):
    def __init__(self):
        super().__init__('udp_twist_teleop_publisher')

        # ---- Network Parameters ----
        self.declare_parameter('udp_ip',           "0.0.0.0") 
        self.declare_parameter('udp_port',         5005)      
        self.declare_parameter('rate',             30.0)
        
        # Smoothing and Deadzones
        self.declare_parameter('alpha',            0.2)   
        self.declare_parameter('dead_zone',        0.15)  # Radians (~8.5 degrees)
        
        # Velocity Scaling 
        self.declare_parameter('vel_scale_x',      5.0) 
        self.declare_parameter('vel_scale_y',      5.0) 
        self.declare_parameter('vel_scale_z',      5.0) 
        self.declare_parameter('max_linear_vel',   2.5)

        # Load parameters
        self.udp_ip       = self.get_parameter('udp_ip').value
        self.udp_port     = self.get_parameter('udp_port').value
        self.rate         = self.get_parameter('rate').value
        self.alpha        = self.get_parameter('alpha').value
        self.dead_zone    = self.get_parameter('dead_zone').value
        self.v_scale_x    = self.get_parameter('vel_scale_x').value
        self.v_scale_y    = self.get_parameter('vel_scale_y').value
        self.v_scale_z    = self.get_parameter('vel_scale_z').value
        self.max_lin      = self.get_parameter('max_linear_vel').value

        # ---- Setup UDP Socket ----
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.setblocking(False) # Prevents the ROS 2 timer from freezing

        # ---- Publishers ----
        self.twist_pub = self.create_publisher(Twist, '/twist_controller/commands', 10)
        self.arrow_pub = self.create_publisher(Marker, '/teleop_velocity_arrow', 10)

        # ---- TF2 Listener ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- Gripper Client ----
        self.gripper_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')
        self.last_gripper_state = None 

        # Filtered velocity state (for EMA)
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0

        # ---- Timer ----
        self.timer = self.create_timer(1.0 / self.rate, self._timer_cb)

        self.get_logger().info('========================================')
        self.get_logger().info(f'  UDP Teleop Ready on Port {self.udp_port}')
        self.get_logger().info('  Mapping:')
        self.get_logger().info('    Pitch (Fwd/Back) -> Robot Z (Up/Down)')
        self.get_logger().info('    Roll  (L/R Tilt) -> Robot X (Fwd/Back)')
        self.get_logger().info('    Yaw   (Rotate)   -> Robot Y (Left/Right)')
        self.get_logger().info('========================================')

    def _timer_cb(self):
        # 1. Drain the UDP buffer to get the absolute newest packet
        raw_data = None
        try:
            while True:
                data, addr = self.sock.recvfrom(1024)
                raw_data = data
        except BlockingIOError:
            pass # Buffer is empty, move on

        pitch, roll, yaw = 0.0, 0.0, 0.0
        
        # 2. Parse the UDP Data (JSON + Quaternion to Euler)
        if raw_data is not None:
            try:
                decoded = raw_data.decode('utf-8').strip()
                
                # If multiple packets clumped together, grab the last one
                if '}{' in decoded:
                    decoded = '{' + decoded.split('}{')[-1]

                data_dict = json.loads(decoded)
                
                # Verify it is the Rotation Vector
                if data_dict.get("type") == "android.sensor.rotation_vector":
                    vals = data_dict.get("values", [])
                    if len(vals) >= 4:
                        qx, qy, qz, qw = vals[0], vals[1], vals[2], vals[3]

                        # Convert Quaternion to Euler Angles
                        sinr_cosp = 2 * (qw * qx + qy * qz)
                        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
                        roll = math.atan2(sinr_cosp, cosr_cosp)

                        sinp = 2 * (qw * qy - qz * qx)
                        if abs(sinp) >= 1:
                            pitch = math.copysign(math.pi / 2, sinp) 
                        else:
                            pitch = math.asin(sinp)

                        siny_cosp = 2 * (qw * qz + qx * qy)
                        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
                        yaw = math.atan2(siny_cosp, cosy_cosp)
                        
            except json.JSONDecodeError:
                pass # Ignore torn packets
            except Exception as e:
                self.get_logger().error(f"UDP Parse Error: {e}")

        # 3. Apply Dead-Zone (If phone is mostly flat, force to 0.0)
        if abs(pitch) < self.dead_zone: pitch = 0.0
        if abs(roll)  < self.dead_zone: roll = 0.0
        if abs(yaw)   < self.dead_zone: yaw = 0.0

        # 4. Map Phone Axes to Robot Velocities 
        target_vz = pitch * self.v_scale_z  # Tilt Fwd/Back -> Moves Up/Down
        target_vx = roll  * self.v_scale_x  # Tilt L/R      -> Moves Fwd/Back
        target_vy = yaw   * self.v_scale_y  # Rotate Flat   -> Moves L/R

        # 5. Apply Exponential Moving Average (Shock Absorber)
        self.current_vx = (self.alpha * target_vx) + ((1 - self.alpha) * self.current_vx)
        self.current_vy = (self.alpha * target_vy) + ((1 - self.alpha) * self.current_vy)
        self.current_vz = (self.alpha * target_vz) + ((1 - self.alpha) * self.current_vz)

        # 6. Build and publish the Twist message
        twist_msg = Twist()
        twist_msg.linear.x = np.clip(self.current_vx, -self.max_lin, self.max_lin)
        twist_msg.linear.y = np.clip(self.current_vy, -self.max_lin, self.max_lin)
        twist_msg.linear.z = np.clip(self.current_vz, -self.max_lin, self.max_lin)
        
        self.twist_pub.publish(twist_msg)

        # 7. Visualize Arrow in RViz
        is_moving = abs(self.current_vx) > 0.01 or abs(self.current_vy) > 0.01 or abs(self.current_vz) > 0.01
        self._publish_arrow(twist_msg, is_moving)

        # 8. Gripper Trigger (Currently a placeholder)
        # TODO: You can map this boolean to a specific tilt, or a ROS subscriber!
        trigger_gripper_close = False 
        
        if trigger_gripper_close != self.last_gripper_state:
            self._send_gripper_command(close=trigger_gripper_close)
            self.last_gripper_state = trigger_gripper_close

    def _publish_arrow(self, twist_msg, is_moving):
        arrow = Marker()
        arrow.header.frame_id = 'base_link'
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.ns = 'teleop_direction'
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD

        start_pt = Point(x=0.0, y=0.0, z=0.5) 
        visual_scale = 2.0 
        end_pt = Point()
        end_pt.x = start_pt.x + (twist_msg.linear.x * visual_scale)
        end_pt.y = start_pt.y + (twist_msg.linear.y * visual_scale)
        end_pt.z = start_pt.z + (twist_msg.linear.z * visual_scale)
        arrow.points = [start_pt, end_pt]

        arrow.scale.x = 0.02
        arrow.scale.y = 0.04
        arrow.scale.z = 0.04
        arrow.color.r = 0.0
        arrow.color.g = 1.0
        arrow.color.b = 1.0
        arrow.color.a = 0.8 if is_moving else 0.0 

        self.arrow_pub.publish(arrow)

    def _send_gripper_command(self, close=True):
        if not self.gripper_client.wait_for_server(timeout_sec=0.1):
            return

        goal = GripperCommand.Goal()
        goal.command.position = 0.4867 if close else 0.0713
        
        self.get_logger().info(f'Sending Gripper Goal: {"CLOSE" if close else "OPEN"}')
        self.gripper_client.send_goal_async(goal)

def main(args=None):
    rclpy.init(args=args)
    node = UdpTwistTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.twist_pub.publish(Twist()) # Force stop on shutdown
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()