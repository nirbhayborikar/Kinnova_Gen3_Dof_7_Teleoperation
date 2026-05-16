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
        self.declare_parameter('tag_frame', 'tag36h11_0')    # Default AprilTag ID 0

        # --- Angular Parameters (Radians) ---
        self.declare_parameter('max_angular_speed', 1.0) # Rad/s (~57 deg/s)
        self.declare_parameter('angular_dead_zone', 0.1) # Rad/s (~5.7 deg/s)




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

        # angular parameter 
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.angular_dead_zone = self.get_parameter('angular_dead_zone').value





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




        # the robot current angular memory

        # Angular memory
        self.prev_roll = None
        self.prev_pitch = None
        self.prev_yaw = None
        
        # Filtered angular states
        self.current_wx = 0.0
        self.current_wy = 0.0
        self.current_wz = 0.0



        self.timer = self.create_timer(1.0 / self.rate, self._timer_cb)

        self.get_logger().info("=========================================")
        self.get_logger().info("  AprilTag 3D Teleop Activated!          ")
        self.get_logger().info("  Show the tag to the camera to move.    ")
        self.get_logger().info("=========================================")

    # Light issue / Tag is getting detected but flickering or jumping
    
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


            # 1. Grab Raw Quaternions
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w





        # 2. Convert to human-readable Euler Angles (Roll, Pitch, Yaw)
            sinr_cosp = 2 * (qw * qx + qy * qz)
            cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
            roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))


            sinp = 2 * (qw * qy - qz * qx)
            pitch = math.degrees(math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp))

            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

            # 3. Print to terminal (Throttled to twice a second so it doesn't flood your screen!)
            self.get_logger().info(
                f"TAG [Pos] X:{curr_x:.2f}m Y:{curr_y:.2f}m Z:{curr_z:.2f}m | "
                f"[Rot] Roll:{roll:.1f}° Pitch:{pitch:.1f}° Yaw:{yaw:.1f}°",
                throttle_duration_sec=0.5 
            )


            # Tare memory on first sight (or after being lost)(Anchor position)
            # If this is the first frame we see, save it and skip math
            if self.prev_x is None:
                # Set the new Anchor Point
                self.prev_x, self.prev_y, self.prev_z = curr_x, curr_y, curr_z
                self.prev_roll, self.prev_pitch, self.prev_yaw = roll, pitch, yaw
                self.prev_time = current_time
                self.get_logger().info("--- TAG ACQUIRED! ---")
                return

            # 2. Derivative Calculus (Velocity = Change in Distance / Time)
            dt = (current_time - self.prev_time).nanoseconds / 1e9
            if dt <= 0: return

            raw_vx = (curr_x - self.prev_x) / dt
            raw_vy = (curr_y - self.prev_y) / dt
            raw_vz = (curr_z - self.prev_z) / dt


 ########################### roll pitch ya
 # If this is the first frame, save angles and skip math
            if self.prev_roll is None:
                self.prev_roll, self.prev_pitch, self.prev_yaw = roll, pitch, yaw
                # ... (save position memory as well) ...
                return

            # 1. Find the change in angle (and fix the 360-degree snap problem)
            d_roll = roll - self.prev_roll
            d_pitch = pitch - self.prev_pitch
            d_yaw = yaw - self.prev_yaw

            # Fix angle wrapping (so math doesn't freak out if you cross from 180 to -180)
            if d_roll > 180: d_roll -= 360
            elif d_roll < -180: d_roll += 360
            if d_pitch > 180: d_pitch -= 360
            elif d_pitch < -180: d_pitch += 360
            if d_yaw > 180: d_yaw -= 360
            elif d_yaw < -180: d_yaw += 360

            # 2. Derivative Calculus for Angular Velocity (Degrees per second)
            # We convert to Radians because ROS Twist messages require Radians!
            raw_wx = math.radians(d_roll) / dt
            raw_wy = math.radians(d_pitch) / dt
            raw_wz = math.radians(d_yaw) / dt



            # Filter the Angular Noise!
            active_wx = self.apply_smooth_deadzone(raw_wx, self.angular_dead_zone)
            active_wy = self.apply_smooth_deadzone(raw_wy, self.angular_dead_zone)
            active_wz = self.apply_smooth_deadzone(raw_wz, self.angular_dead_zone)



            # Angular EMA (Assuming Aligned Camera/Robot Frames)
            self.current_wx = (self.alpha * (active_wx * self.scale)) + ((1 - self.alpha) * self.current_wx)
            self.current_wy = (self.alpha * (active_wy * self.scale)) + ((1 - self.alpha) * self.current_wy)
            self.current_wz = (self.alpha * (active_wz * self.scale)) + ((1 - self.alpha) * self.current_wz)



            # ==========================================
            # MAPPING & ALPHA FILTERING (EMA)
            # ==========================================











            # 3. Apply the Ghost Filter (Deadzone)
            active_vx = self.apply_smooth_deadzone(raw_vx, self.dead_zone)
            active_vy = self.apply_smooth_deadzone(raw_vy, self.dead_zone)
            active_vz = self.apply_smooth_deadzone(raw_vz, self.dead_zone)

            # 4. MAPPING (1-to-1 Aligned Frames)
            target_vx = active_vx * self.scale
            target_vy = active_vy * self.scale
            target_vz = active_vz * self.scale

            # 5. Apply the Low-Pass Filter (Alpha EMA) # pROBLEM SOLVED WITH EMA 
            """ Even with the Anchor setting, sometimes the AprilTag algorithm calculates a single 
            bad frame where the tag "teleports" 5 centimeters due to a glitch. We solve this using a
            Low-Pass Filter (Alpha EMA), which acts like a physical shock absorber."""
            self.current_vx = (self.alpha * target_vx) + ((1 - self.alpha) * self.current_vx)
            self.current_vy = (self.alpha * target_vy) + ((1 - self.alpha) * self.current_vy)
            self.current_vz = (self.alpha * target_vz) + ((1 - self.alpha) * self.current_vz)











            # 3. MAPPING: Translate Camera Frame to Robot Frame
            twist = Twist()


            # oreintation
            # Clamp Angular
            twist.angular.x = max(min(self.current_wx, self.max_angular), -self.max_angular)
            twist.angular.y = max(min(self.current_wy, self.max_angular), -self.max_angular)
            twist.angular.z = max(min(self.current_wz, self.max_angular), -self.max_angular)

            
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
            self.prev_roll, self.prev_pitch, self.prev_yaw = roll, pitch, yaw  # update the angle

            self.prev_time = current_time

        except TransformException:
            # SAFETY CATCH: If the camera loses sight of the tag (e.g., your hand covers it),
            # stop the robot instantly and reset memory.
            # ==========================================
            # SAFEGUARD: THE WATCHDOG TIMEOUT
            # ==========================================
            """ If the tag goes out of frame, the robot needs to stop immediately so it doesn't
            keep flying off into the wall based on your last known speed. We solve this using the Watchdog Timer."""

            time_since_seen = (current_time - self.last_seen_time).nanoseconds / 1e9
            
            # If the tag has been hidden longer than the timeout # tag lost slams breaks
            if time_since_seen > self.timeout and self.prev_x is not None:
                # 1. Slam the brakes to 0,0,0
                self.twist_pub.publish(Twist()) 
                
                # 2. Erase memory so the calculus doesn't jump when the tag returns
                self.prev_x = None 
                self.prev_roll = None #  for angle
                self.current_vx, self.current_vy, self.current_vz = 0.0, 0.0, 0.0
                self.current_wx, self.current_wy, self.current_wz = 0.0, 0.0, 0.0
                
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