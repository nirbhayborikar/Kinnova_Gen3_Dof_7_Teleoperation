#!/usr/bin/env python3
"""
UDP IMU Teleop Node for Kinova Gen3 7-DOF.
Listens for Phone Orientation Data (Rotation Vector) via Sensagram JSON over WiFi.

rotate phone  arm goes in z direction  anticlockwise goes up , clockwise goes down

tilt left goes backward tilt right goes forward

tilt forward goes down, tilt backward goes up in z direction
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

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


class UdpTwistTeleopNode(Node):
    def __init__(self):
        super().__init__('udp_twist_teleop_publisher')

        # ---- Network Parameters ----
        self.declare_parameter('udp_ip',           "0.0.0.0") 
        self.declare_parameter('udp_port',         5005)      
        self.declare_parameter('rate',             30.0)
        
        # ---- STRICTER PARAMETERS FOR JITTER ----
        self.declare_parameter('alpha',            0.1)   # Lowered from 0.2 -> Absorbs hand shakes much better
        self.declare_parameter('dead_zone',        0.29)  # Increased from 0.10 -> Phone must tilt ~11.5 degrees before waking up
        
        # Velocity Scaling 
        self.declare_parameter('vel_scale_x',      0.1) 
        self.declare_parameter('vel_scale_y',      0.1) 
        self.declare_parameter('vel_scale_z',      0.1) 
        self.declare_parameter('max_linear_vel',   0.1)

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
        self.sock.setblocking(False) 

        # ---- Publishers ----
        self.twist_pub = self.create_publisher(Twist, '/twist_controller/commands', 10)
        self.arrow_pub = self.create_publisher(Marker, '/teleop_velocity_arrow', 10)

        # ---- TF2 Listener ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Filtered velocity state (for EMA)
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0
        
        # Persistent IMU states
        self.latest_pitch = 0.0
        self.latest_roll = 0.0
        self.latest_yaw = 0.0

        # Tare states
        self.initial_pitch = None
        self.initial_roll = None
        self.initial_yaw = None

        # ---- SAFETY WATCHDOG TIMER ----
        # Tracks the exact moment we last received a valid packet
        self.last_valid_data_time = self.get_clock().now()

        # ---- Timer ----
        self.timer = self.create_timer(1.0 / self.rate, self._timer_cb)

        self.get_logger().info('========================================')
        self.get_logger().info('  STRICT UDP Teleop + Safety Watchdog   ')
        self.get_logger().info('========================================')

    def apply_smooth_deadzone(self, value, deadzone):
        """Prevents sudden jerking when leaving the deadzone by scaling from 0.0 smoothly."""
        if abs(value) <= deadzone:
            return 0.0
        # If value is 0.25 and deadzone is 0.20, it returns 0.05
        return (abs(value) - deadzone) * math.copysign(1.0, value)

    def _timer_cb(self):
        # 1. Drain the UDP buffer
        try:
            while True:
                data, addr = self.sock.recvfrom(1024)
                decoded = data.decode('utf-8').strip()
                
                if '}{' in decoded:
                    decoded = '{' + decoded.split('}{')[-1]

                try:
                    data_dict = json.loads(decoded)
                    
                    if data_dict.get("type") == "android.sensor.rotation_vector":
                        # --- WATCHDOG RESET ---
                        # We got valid data! Update the clock.
                        self.last_valid_data_time = self.get_clock().now()
                        
                        vals = data_dict.get("values", [])
                        if len(vals) >= 4:
                            qx, qy, qz, qw = vals[0], vals[1], vals[2], vals[3]

                            sinr_cosp = 2 * (qw * qx + qy * qz)
                            cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
                            self.latest_roll = math.atan2(sinr_cosp, cosr_cosp)

                            sinp = 2 * (qw * qy - qz * qx)
                            if abs(sinp) >= 1:
                                self.latest_pitch = math.copysign(math.pi / 2, sinp) 
                            else:
                                self.latest_pitch = math.asin(sinp)

                            siny_cosp = 2 * (qw * qz + qx * qy)
                            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
                            self.latest_yaw = math.atan2(siny_cosp, cosy_cosp)
                            
                except json.JSONDecodeError:
                    pass 
                except Exception as e:
                    self.get_logger().error(f"UDP Parse Error: {e}")
                    
        except BlockingIOError:
            pass 

        # ==========================================
        # SAFETY CHECK: THE WATCHDOG
        # ==========================================
        # Calculate how long it has been since the phone last spoke to us
        time_since_last_packet = (self.get_clock().now() - self.last_valid_data_time).nanoseconds / 1e9
        
        # If the phone stopped streaming, or WiFi lagged for more than 0.5 seconds:
        if time_since_last_packet > 0.2:
            # INSTANTLY STOP THE ROBOT
            self.current_vx = 0.0
            self.current_vy = 0.0
            self.current_vz = 0.0
            self.twist_pub.publish(Twist()) # Send absolute zero
            self.get_logger().warning("Stream STOPPED or WiFi lost! Brakes applied.", throttle_duration_sec=1.5)
            return # Skip all movement math below!

        # ==========================================
        # 2. FULL 3-AXIS TARE & SMOOTH DEAD-ZONE
        # ==========================================
        if self.initial_pitch is None and self.latest_pitch != 0.0:
            self.initial_pitch = self.latest_pitch
            self.initial_roll = self.latest_roll
            self.initial_yaw = self.latest_yaw
            self.get_logger().info(f"--- TARE COMPLETE ---")

        rel_pitch, rel_roll, rel_yaw = 0.0, 0.0, 0.0

        if self.initial_pitch is not None:
            rel_pitch = self.latest_pitch - self.initial_pitch
            rel_roll  = self.latest_roll  - self.initial_roll
            rel_yaw   = self.latest_yaw   - self.initial_yaw

            while rel_yaw > math.pi: rel_yaw -= 2 * math.pi
            while rel_yaw < -math.pi: rel_yaw += 2 * math.pi
            while rel_pitch > math.pi: rel_pitch -= 2 * math.pi
            while rel_pitch < -math.pi: rel_pitch += 2 * math.pi
            while rel_roll > math.pi: rel_roll -= 2 * math.pi
            while rel_roll < -math.pi: rel_roll += 2 * math.pi

        # Use the new SMOOTH deadband logic
        final_pitch = self.apply_smooth_deadzone(rel_pitch, self.dead_zone)
        final_roll  = self.apply_smooth_deadzone(rel_roll, self.dead_zone)
        final_yaw   = self.apply_smooth_deadzone(rel_yaw, self.dead_zone)

        # ==========================================
        # 3. KINEMATIC MAPPING & PUBLISHING
        # ==========================================
        target_vz = final_pitch * self.v_scale_z  
        target_vx = final_roll  * self.v_scale_x  
        target_vy = final_yaw   * self.v_scale_y

        self.current_vx = (self.alpha * target_vx) + ((1 - self.alpha) * self.current_vx)
        self.current_vy = (self.alpha * target_vy) + ((1 - self.alpha) * self.current_vy)
        self.current_vz = (self.alpha * target_vz) + ((1 - self.alpha) * self.current_vz)

        twist_msg = Twist()
        twist_msg.linear.x = np.clip(self.current_vx, -self.max_lin, self.max_lin)
        twist_msg.linear.y = np.clip(self.current_vy, -self.max_lin, self.max_lin)
        twist_msg.linear.z = np.clip(self.current_vz, -self.max_lin, self.max_lin)

        if abs(twist_msg.linear.x) > 0.001 or abs(twist_msg.linear.y) > 0.001 or abs(twist_msg.linear.z) > 0.001:
            self.get_logger().info(f"Twist -> X: {twist_msg.linear.x:.2f} | Y: {twist_msg.linear.y:.2f} | Z: {twist_msg.linear.z:.2f}", throttle_duration_sec=0.25)
        
        self.twist_pub.publish(twist_msg)

        is_moving = abs(self.current_vx) > 0.01 or abs(self.current_vy) > 0.01 or abs(self.current_vz) > 0.01
        self._publish_arrow(twist_msg, is_moving)

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


def main(args=None):
    rclpy.init(args=args)
    node = UdpTwistTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.twist_pub.publish(Twist())
        except Exception:
            pass 
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()