#!/usr/bin/env python3
"""
AprilTag 3D Linear Teleop Node for Kinova Gen3.
Tracks absolute position via TF2 and uses derivatives to calculate Twist velocities.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, TransformException
import math

class AprilTagTeleop(Node):
    def __init__(self):
        super().__init__('apriltag_teleop')

        # --- Parameters ---
        self.declare_parameter('rate', 30.0)           # Loop rate (Hz)
        self.declare_parameter('speed_scale', 2.0)     # Sensitivity multiplier
        self.declare_parameter('max_speed', 0.1)       # Safety speed limit (m/s)
        self.declare_parameter('camera_frame', 'camera_link') 
        self.declare_parameter('tag_frame', 'tag36h11_12')    # Default AprilTag ID 0

        self.rate = self.get_parameter('rate').value
        self.scale = self.get_parameter('speed_scale').value
        self.max_speed = self.get_parameter('max_speed').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.tag_frame = self.get_parameter('tag_frame').value



        # --- SAFEGUARD PARAMETERS ---
        # 1. Alpha: 0.01 (Heavy lag, very smooth) to 1.0 (Zero lag, very jittery)
        self.declare_parameter('alpha', 0.15) 
        # 2. Deadzone: Ignore velocities smaller than this (m/s)
        self.declare_parameter('dead_zone', 0.15) 
        # 3. Watchdog Timeout: Seconds before brakes apply when tag is hidden
        self.declare_parameter('tag_timeout', 0.25) 

        self.rate = self.get_parameter('rate').value
        self.scale = self.get_parameter('speed_scale').value
        self.max_speed = self.get_parameter('max_speed').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.tag_frame = self.get_parameter('tag_frame').value
        self.alpha = self.get_parameter('alpha').value
        self.dead_zone = self.get_parameter('dead_zone').value
        self.timeout = self.get_parameter('tag_timeout').value





        # --- TF2 Setup (This is how we find the tag) ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Publishers ---
        self.twist_pub = self.create_publisher(Twist, '/twist_controller/commands', 10)

        # --- Memory for Calculus (Derivatives) ---
        self.prev_time = self.get_clock().now()
        self.last_seen_time = self.get_clock().now()

        #  the robot current position
        self.prev_x = None
        self.prev_y = None
        self.prev_z = None

        # Filtered output states
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0

        self.timer = self.create_timer(1.0 / self.rate, self._timer_cb)

        self.get_logger().info("=========================================")
        self.get_logger().info("  AprilTag 3D Teleop Activated!          ")
        self.get_logger().info("  Show the tag to the camera to move.    ")
        self.get_logger().info("=========================================")


    def apply_smooth_deadzone(self, value, deadzone):
        """Prevents sudden jerking when leaving the deadzone."""
        if abs(value) <= deadzone: # 0.05
            return 0.0
        return (abs(value) - deadzone) * math.copysign(1.0, value)



    def _timer_cb(self):
        current_time = self.get_clock().now()

        try:
            # 1. Ask ROS 2 where the tag is right NOW
            t = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.tag_frame,
                rclpy.time.Time() # Get the newest available frame
            )

            # FEED THE WATCHDOG: We saw the tag, update the clock!
            self.last_seen_time = current_time

            # Extract absolute Cartesian Position (in meters)
            curr_x = t.transform.translation.x
            curr_y = t.transform.translation.y
            curr_z = t.transform.translation.z

            # Tare memory on first sight (or after being lost)(Anchor position)
            # If this is the first frame we see, save it and skip math
            if self.prev_x is None:
                self.prev_x, self.prev_y, self.prev_z = curr_x, curr_y, curr_z
                self.prev_time = current_time
                self.get_logger().info("--- TAG ACQUIRED! ---")
                return

            # 2. Derivative Calculus (Velocity = Change in Distance / Time)
            dt = (current_time - self.prev_time).nanoseconds / 1e9
            if dt <= 0: return

            raw_vx = (curr_x - self.prev_x) / dt
            raw_vy = (curr_y - self.prev_y) / dt
            raw_vz = (curr_z - self.prev_z) / dt



            # 3. Apply the Ghost Filter (Deadzone)
            active_vx = self.apply_smooth_deadzone(raw_vx, self.dead_zone)
            active_vy = self.apply_smooth_deadzone(raw_vy, self.dead_zone)
            active_vz = self.apply_smooth_deadzone(raw_vz, self.dead_zone)

            # 4. MAPPING (1-to-1 Aligned Frames)
            target_vx = active_vx * self.scale
            target_vy = active_vy * self.scale
            target_vz = active_vz * self.scale

            # 5. Apply the Low-Pass Filter (Alpha EMA)
            self.current_vx = (self.alpha * target_vx) + ((1 - self.alpha) * self.current_vx)
            self.current_vy = (self.alpha * target_vy) + ((1 - self.alpha) * self.current_vy)
            self.current_vz = (self.alpha * target_vz) + ((1 - self.alpha) * self.current_vz)











            # 3. MAPPING: Translate Camera Frame to Robot Frame
            twist = Twist()
            
            # Robot Up/down (z) <- camera Z (Because both point UP!)
            # Moving tag up makes it positive. Robot up is positive
            twist.linear.z = max(min(self.current_vz, self.max_speed), -self.max_speed)
            
            # Robot Forward/Backward (X) <- Camera Forward/Backward (Red Axis)
            # Moving the tag towards you makes the robot move towards you.
            twist.linear.x = max(min(self.current_vx, self.max_speed), -self.max_speed)

            # Robot Left/Right (Y) <- Camera Left/Right (Green Axis)
            # Moving the tag to the left makes the robot move left.
            twist.linear.y = max(min(self.current_vy, self.max_speed), -self.max_speed)


            # 5. Publish to Robot
            self.twist_pub.publish(twist)

            # 6. Save memory for the next loop
            self.prev_x, self.prev_y, self.prev_z = curr_x, curr_y, curr_z
            self.prev_time = current_time

        except TransformException:
            # SAFETY CATCH: If the camera loses sight of the tag (e.g., your hand covers it),
            # stop the robot instantly and reset memory.
            # ==========================================
            # SAFEGUARD: THE WATCHDOG TIMEOUT
            # ==========================================
            time_since_seen = (current_time - self.last_seen_time).nanoseconds / 1e9
            
            # If the tag has been hidden longer than the timeout
            if time_since_seen > self.timeout and self.prev_x is not None:
                # 1. Slam the brakes to 0,0,0
                self.twist_pub.publish(Twist()) 
                
                # 2. Erase memory so the calculus doesn't jump when the tag returns
                self.prev_x = None 
                self.current_vx, self.current_vy, self.current_vz = 0.0, 0.0, 0.0
                
                self.get_logger().warning("TAG LOST! Brakes applied.", throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.twist_pub.publish(Twist()) 
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()