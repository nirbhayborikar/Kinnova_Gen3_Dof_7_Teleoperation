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

# Do action client
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

import cv2
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# MediaPipe helper variables (Must be AFTER importing mediapipe!)
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles


class AprilTagTeleop(Node):
    def __init__(self):
        super().__init__('apriltag_teleop')

        # --- Parameters ---
        self.declare_parameter('rate', 30.0)           # Loop rate (Hz)
        self.declare_parameter('speed_scale', 2.0)     # Sensitivity multiplier
        self.declare_parameter('max_speed', 0.05)       # Safety speed limit (m/s)
        self.declare_parameter('camera_frame', 'camera_link') 
        self.declare_parameter('tag_frame', 'tag36h11_0')    # Default AprilTag ID 0

        self.rate = self.get_parameter('rate').value
        self.scale = self.get_parameter('speed_scale').value
        self.max_speed = self.get_parameter('max_speed').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.tag_frame = self.get_parameter('tag_frame').value



        # --- SAFEGUARD PARAMETERS ---
        # 1. Alpha: 0.01 (Heavy lag, very smooth) to 1.0 (Zero lag, very jittery)
        self.declare_parameter('alpha', 0.10) 
        # 2. Deadzone: Ignore velocities smaller than this (m/s)
        self.declare_parameter('dead_zone', 0.20) 
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



        # ==========================================
        # WORKSPACE ENVELOPE (Virtual Walls in Meters)
        # ==========================================
        # 84  longest, 27 y axis  64 total smallest
        # Z: Prevent smashing the table (min 10cm) or hitting the ceiling
        self.declare_parameter('z_min', 0.30) # in m 
        self.declare_parameter('z_max', 0.50)
        
        # X: Forward/Backward limits relative to robot base
        self.declare_parameter('x_min', 0.60) # Don't crash into its own base
        self.declare_parameter('x_max', 1.0) # Max reach
        
        # Y: Left/Right limits
        self.declare_parameter('y_min', -0.40)
        self.declare_parameter('y_max', 0.40)

        # Load values into memory
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value
        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value
        self.y_min = self.get_parameter('y_min').value
        self.y_max = self.get_parameter('y_max').value




        # ==========================================
        # GRIPPER ACTION CLIENT SETUP
        # ==========================================
        self.gripper_client = ActionClient(
            self, 
            GripperCommand, 
            '/robotiq_gripper_controller/gripper_cmd'
        )
        self.last_gripper_state = None # Tracks if it is currently open or closed




        # ==========================================
        # MEDIAPIPE & CAMERA SUBSCRIPTION SETUP
        # ==========================================
        self.bridge = CvBridge()
        
        # Initialize the MediaPipe AI
        self.mp_hands = mp.solutions.hands.Hands(
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7,
            max_num_hands=1 # Only track 1 hand so it doesn't get confused!
        )
        
        # Subscribe to the camera so _image_cb actually runs!
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/color/image_raw', # Make sure this matches your camera topic!
            self._image_cb, 
            10
        )




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


        # ==========================================
        # NEW: Robot physical position (For the Virtual Wall)
        # ==========================================
        self.end_effector_x = 0.0
        self.end_effector_y = 0.0
        self.end_effector_z = 0.0


        # Timestamp memory to detect hidden tags
        self.last_tf_time = 0.0


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




    def _send_gripper_command(self, close=True):
        """Sends the exact motor positions to the Robotiq Gripper."""
        if not self.gripper_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn('Gripper action server not ready')
            return

        goal = GripperCommand.Goal()
        # 0.4867 = Closed | 0.0713 = Open (From your physical testing!)
        goal.command.position = 0.4867 if close else 0.0713
        
        self.get_logger().info(f'Sending Gripper Goal: {"CLOSE" if close else "OPEN"}')
        self.gripper_client.send_goal_async(goal)






    def _image_cb(self, msg):
            """Processes the raw camera image using MediaPipe to detect finger pinches."""
            try:
                # 1. Convert ROS Image to OpenCV format
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                
                # 2. MediaPipe requires RGB color space
                rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                
                # 3. Run the AI model
                results = self.mp_hands.process(rgb_image)
                
                # 4. If a hand is found...
                if results.multi_hand_landmarks:
                    
                    # --- NEW: Draw the skeleton on the image! ---
                    for hand_landmarks in results.multi_hand_landmarks:
                        mp_drawing.draw_landmarks(
                            cv_image,
                            hand_landmarks,
                            mp_hands.HAND_CONNECTIONS,
                            mp_drawing_styles.get_default_hand_landmarks_style(),
                            mp_drawing_styles.get_default_hand_connections_style())

                    hand = results.multi_hand_landmarks[0]
                    
                    # Get coordinates of Thumb Tip (4) and Index Tip (8)
                    thumb = hand.landmark[mp_hands.HandLandmark.THUMB_TIP]
                    index = hand.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
                    
                    # 5. Calculate 2D distance between Thumb (4) and Index (8)
                    pinch_dist = math.hypot(thumb.x - index.x, thumb.y - index.y)
                    
                    # 6. TRIGGER YOUR GRIPPER ACTION CLIENT
                    if pinch_dist < 0.05 and self.last_gripper_state != True:
                        self.last_gripper_state = True
                        #self._send_gripper_command(close=True)
                        
                    elif pinch_dist > 0.10 and self.last_gripper_state != False:
                        self.last_gripper_state = False
                        #self._send_gripper_command(close=False)

                    # --- NEW: Visual Status Text ---
                    state_text = "CLOSED (Pinching)" if self.last_gripper_state else "OPEN"
                    color = (0, 0, 255) if self.last_gripper_state else (0, 255, 0)
                    cv2.putText(cv_image, f"Gripper: {state_text}", (10, 40), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                else:
                    cv2.putText(cv_image, "No Hand Detected", (10, 40), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (150, 150, 150), 2)

                # --- NEW: SHOW THE VIDEO WINDOW ---
                cv2.imshow("MediaPipe Hand Tracker", cv_image)
                cv2.waitKey(1) # Critical! This tells OpenCV to actually refresh the window

            except Exception as e:
                self.get_logger().error(f"MediaPipe Error: {e}")





    def destroy_node(self):
            """Clean up OpenCV windows when shutting down."""
            cv2.destroyAllWindows()
            super().destroy_node()







    def _timer_cb(self):
        current_time = self.get_clock().now()

        try:
            # 1. Ask ROS 2 where the tag is right NOW
            t = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.tag_frame,
                rclpy.time.Time() # Get the newest available frame
            )

            # ==========================================
            # NEW: THE GHOST FRAME FIX
            # ==========================================
            # Calculate the exact timestamp of the camera frame

            tf_time = t.header.stamp.sec + (t.header.stamp.nanosec / 1e9)
            
            if tf_time == self.last_tf_time:
                # The tag is hidden! TF2 is just repeating the last known position.
                # We raise an error to force the script down to the Watchdog block!
                raise TransformException("Stale Frame - Tag is Hidden!")
                
            self.last_tf_time = tf_time

            # ==========================================

            # FEED THE WATCHDOG: We saw the tag, update the clock!
            self.last_seen_time = current_time


            # ==========================================
            # NEW: Ask ROS 2 where the ROBOT is right NOW
            # ==========================================
            t_robot = self.tf_buffer.lookup_transform(
                'base_link', 
                'grasping_frame', # Note: Kinova might call this 'tool_frame'
                rclpy.time.Time()
            )
            self.end_effector_x = t_robot.transform.translation.x
            self.end_effector_y = t_robot.transform.translation.y
            self.end_effector_z = t_robot.transform.translation.z







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
                self.prev_time = current_time
                self.get_logger().info("--- TAG ACQUIRED! ---")
                return

            # 2. Derivative Calculus (Velocity = Change in Distance / Time)
            dt = (current_time - self.prev_time).nanoseconds / 1e9
            if dt <= 0: return

            raw_vx = (curr_x - self.prev_x) / dt
            raw_vy = (curr_y - self.prev_y) / dt
            raw_vz = (curr_z - self.prev_z) / dt

            # ==========================================
            # NEW: THE TELEPORT CATCHER
            # ==========================================
            # If the calculated speed is physically impossible for a human hand (> 3.0 m/s),
            # it means the camera glitched. We erase the memory and reset the anchor!
            if abs(raw_vx) > 2.0 or abs(raw_vy) > 2.0 or abs(raw_vz) > 2.0:
                self.prev_x = None 
                self.current_vx, self.current_vy, self.current_vz = 0.0, 0.0, 0.0
                return
            # ==========================================


            # 3. Apply the Ghost Filter (Deadzone)
            active_vx = self.apply_smooth_deadzone(raw_vx, self.dead_zone)
            active_vy = self.apply_smooth_deadzone(raw_vy, self.dead_zone)
            active_vz = self.apply_smooth_deadzone(raw_vz, self.dead_zone)

            # 4. MAPPING (1-to-1 Aligned Frames)

    
            target_vx = active_vx * self.scale
            target_vy = active_vy * self.scale
            target_vz = active_vz * self.scale  # x up down  goes to robot z down up

            # 5. Apply the Low-Pass Filter (Alpha EMA) # pROBLEM SOLVED WITH EMA 
            """ Even with the Anchor setting, sometimes the AprilTag algorithm calculates a single 
            bad frame where the tag "teleports" 5 centimeters due to a glitch. We solve this using a
            Low-Pass Filter (Alpha EMA), which acts like a physical shock absorber."""
            self.current_vx = (self.alpha * target_vx) + ((1 - self.alpha) * self.current_vx)
            self.current_vy = (self.alpha * target_vy) + ((1 - self.alpha) * self.current_vy)
            self.current_vz = (self.alpha * target_vz) + ((1 - self.alpha) * self.current_vz)




            # 3. MAPPING: Translate Camera Frame to Robot Frame
            twist = Twist()
            

           
            # 1. 
            twist.linear.z = max(min(-(self.current_vx), self.max_speed), -self.max_speed)

            # the above is confirm camera -x , is robot z (towards us when seeing from front)(pointing outwards as it is end effector.)
            # the position is inverse

            # 2. 
      
            twist.linear.y = max(min(self.current_vz, self.max_speed), -self.max_speed)
            # the above is confirm camera z , is robot y (towards up  going in the ceiling)


            # 3. 
            #twist.linear.x = max(min(-(self.current_vy), self.max_speed), -self.max_speed)     
            # the above is confirm camera  - y , is robot x (lateral axis )
            # the position is inverse thats why -y of camera is +x of robot end effector.


            # ==========================================
            # CORRECTED WORKSPACE ENVELOPE (SAFETY GEOFENCE)
            # ==========================================
            
            # 1. HEIGHT BOUNDARY (Floor/Ceiling)
            # Physical Height is TF2 Z. But your Up/Down motor is Twist Y!
            if self.end_effector_z < self.z_min and twist.linear.y < 0:
                twist.linear.y = 0.0 #  is up down in robot
                self.current_vz = 0.0 # Kill the memory so it doesn't rubber-band
                self.get_logger().warning("⚠️ Z-MIN (Floor) Boundary Reached!", throttle_duration_sec=1.0)
            elif self.end_effector_z > self.z_max and twist.linear.y > 0:
                twist.linear.y = 0.0 
                self.current_vz = 0.0 
                self.get_logger().warning("⚠️ Z-MAX (Ceiling) Boundary Reached!", throttle_duration_sec=1.0)

            # 2. FORWARD/BACKWARD BOUNDARY (Robot Base)
            # Physical Forward/Back is TF2 X. But your Forward/Back motor is Twist Z!
            if self.end_effector_x < self.x_min and twist.linear.z < 0:
                twist.linear.z = 0.0  # z forward backward
                self.current_vx = 0.0 
                self.get_logger().warning("⚠️ X-MIN (Base Crash) Boundary Reached!", throttle_duration_sec=1.0)
            elif self.end_effector_x > self.x_max and twist.linear.z > 0:
                twist.linear.z = 0.0 
                self.current_vx = 0.0 
                self.get_logger().warning("⚠️ X-MAX (Max Reach) Boundary Reached!", throttle_duration_sec=1.0)

            # 3. LEFT/RIGHT BOUNDARY
            # Physical Left/Right is TF2 Y. But your Left/Right motor is Twist X!
            if self.end_effector_y < self.y_min and twist.linear.x < 0:
                twist.linear.x = 0.0 
                self.current_vy = 0.0 
                self.get_logger().warning("⚠️ Y-MIN (Left) Boundary Reached!", throttle_duration_sec=1.0)
            elif self.end_effector_y > self.y_max and twist.linear.x > 0:
                twist.linear.x = 0.0 
                self.current_vy = 0.0 
                self.get_logger().warning("⚠️ Y-MAX (Right) Boundary Reached!", throttle_duration_sec=1.0)

            # 5. Publish to Robot (MUST BE BELOW THE GEOFENCE!)
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
            """ If the tag goes out of frame, the robot needs to stop immediately so it doesn't
            keep flying off into the wall based on your last known speed. We solve this using the Watchdog Timer."""

            time_since_seen = (current_time - self.last_seen_time).nanoseconds / 1e9
            
            # If the tag has been hidden longer than the timeout # tag lost slams breaks
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



