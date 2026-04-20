#!/usr/bin/env python3
"""
UDP IMU Teleop Node for Kinova Gen3 7-DOF.
Listens for Pico 4 VR Controller Data (Rotation Vector) via UDP.

MAPPING:
- Tilt Forward/Backward (Pitch) -> Moves Robot X (Forward/Backward)
- Tilt Left/Right (Roll)        -> Moves Robot Y (Left/Right)
- Rotate Clockwise/Anti (Yaw)   -> Moves Robot Z (Up/Down)
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


class Pico4TeleopNode(Node):
    def __init__(self):
        super().__init__('pico4_teleop_node')

        # ---- Network Parameters ----
        self.declare_parameter('udp_ip',           "0.0.0.0") 
        self.declare_parameter('udp_port',         5005)      
        self.declare_parameter('rate',             30.0)
        
        # ---- STRICTER PARAMETERS FOR PICO 4 ----
        self.declare_parameter('alpha',            0.05) #0.1  # Heavy smoothing for jitter
        self.declare_parameter('dead_zone',        0.15) #0.29  # Must tilt past this threshold to move
        
        # Velocity Scaling (Adjust these if the robot moves too fast or slow)
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

        # Filtered velocity state (for Exponential Moving Average)
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

        # Watchdog Timer
        self.last_valid_data_time = self.get_clock().now()

        # ---- Timer ----
        self.timer = self.create_timer(1.0 / self.rate, self._timer_cb)

        self.get_logger().info('========================================')
        self.get_logger().info('  PICO 4 VR Teleop Activated!           ')
        self.get_logger().info('  Hold controller steady to Tare...     ')
        self.get_logger().info('========================================')

    def apply_smooth_deadzone(self, value, deadzone):
        """Prevents sudden jerking when leaving the deadzone by scaling smoothly from zero."""
        if abs(value) <= deadzone:
            return 0.0
        return (abs(value) - deadzone) * math.copysign(1.0, value)

    def _timer_cb(self):
        current_ros_time = self.get_clock().now()

        # ==========================================
        # 1. DRAIN UDP BUFFER & PARSE QUATERNION
        # ==========================================
        try:
            while True:
                data, addr = self.sock.recvfrom(1024)
                decoded = data.decode('utf-8').strip()
                
                # Fix malformed JSON if multiple packets stuck together
                if '}{' in decoded:
                    decoded = '{' + decoded.split('}{')[-1]

                try:
                    data_dict = json.loads(decoded)
                    
                    if data_dict.get("type") == "android.sensor.rotation_vector":
                        # Feed the Watchdog!
                        self.last_valid_data_time = current_ros_time
                        
                        vals = data_dict.get("values", [])
                        if len(vals) >= 4:
                            qx, qy, qz, qw = vals[0], vals[1], vals[2], vals[3]

                            # Convert Quaternion to Euler (Roll, Pitch, Yaw)
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
        except BlockingIOError:
            pass 

        # ==========================================
        # 2. SAFETY CHECK: THE WATCHDOG
        # ==========================================
        time_since_last_packet = (current_ros_time - self.last_valid_data_time).nanoseconds / 1e9
        
        # If no data for 0.2 seconds, slam the brakes!
        if time_since_last_packet > 0.5:
            self.current_vx, self.current_vy, self.current_vz = 0.0, 0.0, 0.0
            self.twist_pub.publish(Twist()) 
            self.get_logger().warning("Pico 4 Stream LOST! Brakes applied.", throttle_duration_sec=1.5)
            return 

        # ==========================================
        # 3. TARE LOGIC & SMOOTH DEADZONE
        # ==========================================
        if self.initial_pitch is None and self.latest_pitch != 0.0:
            self.initial_pitch = self.latest_pitch
            self.initial_roll = self.latest_roll
            self.initial_yaw = self.latest_yaw
            self.get_logger().info(f"--- PICO 4 TARE COMPLETE ---")

        rel_pitch, rel_roll, rel_yaw = 0.0, 0.0, 0.0

        if self.initial_pitch is not None:
            rel_pitch = self.latest_pitch - self.initial_pitch
            rel_roll  = self.latest_roll  - self.initial_roll
            rel_yaw   = self.latest_yaw   - self.initial_yaw

            # Safely wrap angles to prevent math snapping at 180 degrees
            while rel_yaw > math.pi: rel_yaw -= 2 * math.pi
            while rel_yaw < -math.pi: rel_yaw += 2 * math.pi
            while rel_pitch > math.pi: rel_pitch -= 2 * math.pi
            while rel_pitch < -math.pi: rel_pitch += 2 * math.pi
            while rel_roll > math.pi: rel_roll -= 2 * math.pi
            while rel_roll < -math.pi: rel_roll += 2 * math.pi

        final_pitch = self.apply_smooth_deadzone(rel_pitch, self.dead_zone)
        final_roll  = self.apply_smooth_deadzone(rel_roll, self.dead_zone)
        final_yaw   = self.apply_smooth_deadzone(rel_yaw, self.dead_zone)

        # ==========================================
        # 4. KINEMATIC MAPPING (Industry Standard)
        # ==========================================
        
        # Forward/Backward (X) <- Tilt Forward/Backward (Pitch)
        target_vx = final_pitch * self.v_scale_x  
        
        # Left/Right (Y) <- Tilt Left/Right (Roll)
        target_vy = final_roll * self.v_scale_y  
        
        # Up/Down (Z) <- Rotate Clockwise/Anti-clockwise (Yaw)
        target_vz = final_yaw * self.v_scale_z  

        # ==========================================
        # 5. FILTERING & PUBLISHING
        # ==========================================
        self.current_vx = (self.alpha * target_vx) + ((1 - self.alpha) * self.current_vx)
        self.current_vy = (self.alpha * target_vy) + ((1 - self.alpha) * self.current_vy)
        self.current_vz = (self.alpha * target_vz) + ((1 - self.alpha) * self.current_vz)

        twist_msg = Twist()
        twist_msg.linear.x = np.clip(self.current_vx, -self.max_lin, self.max_lin)
        twist_msg.linear.y = np.clip(self.current_vy, -self.max_lin, self.max_lin)
        twist_msg.linear.z = np.clip(self.current_vz, -self.max_lin, self.max_lin)

        # Log movement for debugging
        if abs(twist_msg.linear.x) > 0.001 or abs(twist_msg.linear.y) > 0.001 or abs(twist_msg.linear.z) > 0.001:
            self.get_logger().info(f"Pico4 -> X: {twist_msg.linear.x:.2f} | Y: {twist_msg.linear.y:.2f} | Z: {twist_msg.linear.z:.2f}", throttle_duration_sec=0.25)
        
        self.twist_pub.publish(twist_msg)

        # Publish visual marker in RViz
        is_moving = abs(self.current_vx) > 0.01 or abs(self.current_vy) > 0.01 or abs(self.current_vz) > 0.01
        self._publish_arrow(twist_msg, is_moving)

    def _publish_arrow(self, twist_msg, is_moving):
        """Publishes a 3D arrow to RViz showing current velocity vector."""
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
    node = Pico4TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Safety catch: send zero velocity on shutdown
        try:
            node.twist_pub.publish(Twist())
        except Exception:
            pass 
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()