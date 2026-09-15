#!/usr/bin/env python3
"""
Gesture-Based Teleoperation Node for Kinova Gen3 7-DOF
Target Controller: picknik_twist_controller/PicknikTwistController

Control Scheme
==============

RIGHT HAND → Robot Motion Activation + Main Cartesian Control
--------------------------------------------------------------
Open hand:
    - Robot in WAITING / STOP mode
    - Publishes zero Twist velocity
    - Motion disabled

Closed fist:
    - Activates robot motion mode
    - Hand movement mapped to robot Cartesian velocity

    Right hand movement:
        Up / Down movement
            → Robot X-axis motion (forward/backward)

        Left / Right movement
            → Robot Y-axis motion (left/right)

    Wrist rotation:
        → Robot angular.z rotation (end-effector roll)

LEFT HAND → Gripper + Secondary Motion Control
-----------------------------------------------

Left hand gestures become active only when:
    Right hand is detected AND right fist is closed
    (acts as a safety enable switch)

Gripper control:

    Left fist closed
        → Close gripper

    Left pinch gesture
        → Close gripper

    Left hand open
        → Open gripper

Robot motion:

    Left hand Up / Down movement
        → Robot Z-axis motion (up/down)

    Left hand Left / Right movement
        → Used as backup Y-axis control

Smart Switching:
----------------
Robot Y-axis movement uses:

    Right hand horizontal motion
        if available

otherwise:

    Left hand horizontal motion

Only one hand controls Y motion at a time.

Additional Features
-------------------

- MediaPipe hand tracking
- Wrist rotation tracking
- Dead-zone filtering for jitter removal
- Exponential Moving Average (EMA) smoothing
- Velocity limiting (safety clipping)
- RViz velocity arrow visualization
- TF2 end-effector position monitoring
- Live camera overlay with status information

Runs continuously at 30 Hz.
"""

import math # Import math functions (sin, cos, sqrt, etc.)
import cv2 # Import OpenCV for image processing
import numpy as np # to store large size of numbers

import rclpy  # ros client library for python
from rclpy.node import Node # Import Node class to create ROS2 nodes
from geometry_msgs.msg import Twist # Import Twist message for robot movement (linear/angular velocity)

# Action client
from rclpy.action import ActionClient # Import ROS2 action client for asynchronous task communication
from control_msgs.action import GripperCommand # Import gripper action message for controlling robot gripper

# Marker visualization messages for RViz
from visualization_msgs.msg import Marker 

# Point message used to define start and end coordinates
# of the velocity arrow visualization
from geometry_msgs.msg import Point

# TF2 components for reading robot coordinate transforms
# Used to obtain the current end-effector position
# (x, y, z) from the TF tree
from tf2_ros import Buffer, TransformListener  # Stores transform data, and Listens for TF updates
from tf2_ros import TransformException  # Handles transform lookup errors

# Custom hand tracking module
# Uses MediaPipe internally to detect hands,
# identify gestures (fist, pinch, open hand),
# track wrist positions and compute rotation data
from .hand_tracker import HandTracker

# Create main ROS2 node class
class TwistTeleopNode(Node):
    # Constructor function will run when object is created
    def __init__(self):
        
        # Initialize ROS2 node with name:
        # /twist_teleop_publisher
        super().__init__('twist_teleop_publisher')


        # =====================================================
        # ROS2 PARAMETERS
        # Parameters allow runtime tuning without changing code
        # =====================================================

        # Camera device index
        # 0 = default webcam


        # ---- Parameters ----
        self.declare_parameter('camera_id',        0)

        # Main processing frequency (Hz)
        # Timer callback runs at this rate
        self.declare_parameter('rate',             30.0)

        # Enable OpenCV display window
        self.declare_parameter('show_camera',      True)
        

        # =====================================================
        # FILTERING PARAMETERS
        # Used to reduce noise and unstable hand movement
        # =====================================================

        # Exponential Moving Average smoothing factor
        # Higher alpha → faster response
        # Lower alpha → smoother motion

        # Smoothing and Deadzones
        self.declare_parameter('alpha',            0.4)


        # Ignore tiny wrist movements
        # Removes hand jitter

        self.declare_parameter('dead_zone',        0.002) 

        # =====================================================
        # VELOCITY SCALING
        # Converts hand movement into robot velocity
        # =====================================================
        
        self.declare_parameter('vel_scale_y',      5.0) # Lateral (left/right)
        self.declare_parameter('vel_scale_z',      5.0) # Vertical (up/down)
        self.declare_parameter('vel_scale_roll',   5.0) # Wrist rotation
        
        # Safety limits for robot speed
        self.declare_parameter('max_linear_vel',   4.0)
        self.declare_parameter('max_angular_vel',  2.0)

        # =====================================================
        # LOAD PARAMETER VALUES
        # Read values from parameter server
        # =====================================================

        cam_id            = self.get_parameter('camera_id').value
        self.rate         = self.get_parameter('rate').value
        self.show_camera  = self.get_parameter('show_camera').value
        self.alpha        = self.get_parameter('alpha').value
        self.dead_zone    = self.get_parameter('dead_zone').value
        self.v_scale_y    = self.get_parameter('vel_scale_y').value
        self.v_scale_z    = self.get_parameter('vel_scale_z').value
        self.v_scale_roll = self.get_parameter('vel_scale_roll').value 
        self.max_lin      = self.get_parameter('max_linear_vel').value
        self.max_ang      = self.get_parameter('max_angular_vel').value

        # =====================================================
        # ROS PUBLISHERS
        # =====================================================

        # Publishes Twist velocity commands
        # Robot controller subscribes here

        self.twist_pub = self.create_publisher(Twist, '/twist_controller/commands', 10)
        
        # Publishes velocity arrows for RViz
        self.arrow_pub = self.create_publisher(Marker, '/teleop_velocity_arrow', 10)


        # =====================================================
        # TF2 LISTENER
        # Reads robot transforms
        # Used to monitor end-effector position
        # =====================================================


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Store current end-effector coordinates
        self.end_effector_x = 0.0
        self.end_effector_y = 0.0
        self.end_effector_z = 0.0

        # =====================================================
        # CAMERA INITIALIZATION
        # Open webcam and set resolution
        # =====================================================

        self.cap = cv2.VideoCapture(cam_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera {cam_id}')
            raise RuntimeError(f'Cannot open camera {cam_id}')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


        # =====================================================
        # HAND TRACKER
        # Uses MediaPipe internally
        # Detects:
        # - left hand
        # - right hand
        # - fist
        # - pinch
        # - wrist rotation
        # =====================================================

        self.tracker = HandTracker(pinch_threshold=0.07)

        # ---- Gripper Action Client.  Sends open/close commands ----
        self.gripper_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')
        self.last_gripper_state = None # Prevent repeated command spam

        # ---- Delta control state ----
        # HAND MOTION MEMORY
        # Stores previous frame positions
        # Used to calculate movement delta

        self.prev_wrist = None
        self.smooth_rot = 0.0
        self.rot_init   = False
        self.prev_left_wrist = None 
        
        # Current smoothed robot velocities
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0
        self.current_wz = 0.0

        # =====================================================
        # TIMER. Executes callback continuously # Example: 1/ 30 Hz → every 33.3 ms
        # =====================================================
        self.timer = self.create_timer(1.0 / self.rate, self._timer_cb)

        self.get_logger().info('========================================')
        self.get_logger().info('  Kinova Twist Teleop (Velocity Mode)')
        self.get_logger().info('  RIGHT HAND: Drive Robot (X, Z, Roll)')
        self.get_logger().info('  LEFT HAND:  Gripper & Strafe (Y)')
        self.get_logger().info('========================================')

    # Timer callback runs repeatedly at fixed rate of 33.3 ms
    def _timer_cb(self):

        # =====================================================
        # READ CAMERA FRAME
        # Timer runs continuously (30 Hz)
        # Capture latest webcam image
        # =====================================================

        ret, frame = self.cap.read()

        # Skip processing if frame capture fails
        if not ret:
            return

        # =====================================================
        # HAND DETECTION
        # Process image using HandTracker
        # Returns:
        # left      -> left hand information
        # right     -> right hand information
        # annotated -> image with landmarks drawn
        # =====================================================

        left, right, annotated = self.tracker.process_frame(frame)
        
        # Get image dimensions for overlays
        h, w = annotated.shape[:2] # 2 means take first two argument (height, width, channels)

        # Create Twist message
        # Stores robot linear/angular velocity
        twist_msg = Twist()
        is_moving = False # Motion state flag

        # =========================================================
        # 1. GRIPPER CONTROL
        #   Gripper only active when:
        # - Right hand detected
        # - Right hand fist closed
        # - Left hand detected
        #
        # Right hand acts as safety switch
        # =========================================================

        if right.detected and right.fist_closed and left.detected:

            # Close gripper if:
            # Left fist closed OR pinch detected
            should_close = left.fist_closed or left.gripper_closed 

            # Send command only if state changed
            # Prevents command spam
            if should_close != self.last_gripper_state:
                self._send_gripper_command(close=should_close)
                self.last_gripper_state = should_close
            
        # =========================================================
        # 2. CALCULATE LEFT HAND MOVEMENT.  
        # Used for: Z-axis movement, Backup Y-axis movement
        # Motion calculated using wrist delta
        # =========================================================
        cam_dx_left = 0.0
        cam_dy_left = 0.0

        if left.detected and left.fist_closed:

            # First frame:
            # initialize previous position
            if self.prev_left_wrist is None:
                self.prev_left_wrist = (left.wrist_x, left.wrist_y)
            else:
                # Compute wrist movement
                raw_dx = left.wrist_x - self.prev_left_wrist[0]
                raw_dy = left.wrist_y - self.prev_left_wrist[1]

                # Apply dead-zone
                # Ignore small hand jitter                
                if abs(raw_dx) >= self.dead_zone: cam_dx_left = raw_dx
                if abs(raw_dy) >= self.dead_zone: cam_dy_left = raw_dy
                
                # Update stored wrist position
                self.prev_left_wrist = (left.wrist_x, left.wrist_y)
        else:
            # Reset when hand disappears
            self.prev_left_wrist = None

        # =========================================================
        # 3. CALCULATE RIGHT HAND & APPLY "SMART SWITCH"
        # Main robot control hand. Controls: X-axis, Y-axis
        # =========================================================
        if right.detected and right.fist_closed:
            is_moving = True

            # First frame initialization
            if self.prev_wrist is None:
                self.prev_wrist = (right.wrist_x, right.wrist_y)
            else:

                # Calculate movement delta
                raw_dx_right = right.wrist_x - self.prev_wrist[0]
                raw_dy_right = right.wrist_y - self.prev_wrist[1]

                # Dead-zone filtering
                cam_dx_right = raw_dx_right if abs(raw_dx_right) >= self.dead_zone else 0.0
                cam_dy_right = raw_dy_right if abs(raw_dy_right) >= self.dead_zone else 0.0



            # =================================================
            # SMART AXIS MAPPING
            # Right vertical:
            #     X movement
            #
            # Left vertical:
            #     Z movement
            #
            # Right horizontal:
            #     Y movement
            #
            # If unavailable:
            #
            # Left horizontal becomes backup
            # =================================================


                # --- THE SMART MAPPING ---
                target_vx = -cam_dy_right * self.v_scale_y 
                target_vz = -cam_dy_left * self.v_scale_z
                
                if abs(cam_dx_right) > 0.0:
                    target_vy = -cam_dx_right * self.v_scale_y
                else:
                    target_vy = -cam_dx_left * self.v_scale_y

                # Apply EMA
                self.current_vx = (self.alpha * target_vx) + ((1 - self.alpha) * self.current_vx)
                self.current_vy = (self.alpha * target_vy) + ((1 - self.alpha) * self.current_vy)
                self.current_vz = (self.alpha * target_vz) + ((1 - self.alpha) * self.current_vz)
                
                # Safety velocity limits
                twist_msg.linear.x = np.clip(self.current_vx, -self.max_lin, self.max_lin)
                twist_msg.linear.y = np.clip(self.current_vy, -self.max_lin, self.max_lin)
                twist_msg.linear.z = np.clip(self.current_vz, -self.max_lin, self.max_lin)
                
                self.prev_wrist = (right.wrist_x, right.wrist_y)
            

        # =================================================
        # WRIST ROTATION CONTROL
        #
        # Converts wrist rotation into
        # angular.z velocity
        # =================================================
            raw_rot = right.rotation
            if not self.rot_init:
                self.smooth_rot = raw_rot
                self.rot_init   = True
            else:
                dr = raw_rot - self.smooth_rot
                while dr >  math.pi: dr -= 2 * math.pi
                while dr < -math.pi: dr += 2 * math.pi
                
                target_wz = dr * self.v_scale_roll 
                self.current_wz = (self.alpha * target_wz) + ((1 - self.alpha) * self.current_wz)
                
                twist_msg.angular.z = np.clip(self.current_wz, -self.max_ang, self.max_ang)
                self.smooth_rot = raw_rot 

        else:
            # Stop robot when fist opens
            self._reset_state()

        # =========================================================
        #  Velocity Arrow Visualization & TF2
        # =========================================================
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

        # =====================================================
        # PUBLISH COMMANDS
        # =====================================================
        self.twist_pub.publish(twist_msg)

        # =====================================================
        # TF2 LOOKUP: Reads current end-effector position, for display/debugging
        # =====================================================
        try:
            t = self.tf_buffer.lookup_transform('base_link', 'grasping_frame', rclpy.time.Time())
            self.end_effector_x = t.transform.translation.x
            self.end_effector_y = t.transform.translation.y
            self.end_effector_z = t.transform.translation.z
        except TransformException:
            pass

        # ── Overlay + display ────────────────────────────────────────
        self._draw_overlay(annotated, right, is_moving, twist_msg, w, h)

        # Display camera
        if self.show_camera:
            cv2.imshow('Kinova Twist Teleop', annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

    # Function to send open/close command to gripper
    def _send_gripper_command(self, close=True):
        # Sends target motor position to Robotiq gripper

        """ Uses ROS2 Action Client to asynchronously
            send gripper open or close commands. 
            Sends the exact motor positions to the Robotiq Gripper."""
        
        # Check if gripper action server is available
        if not self.gripper_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn('Gripper action server not ready') # Show warning if server is unavailable
            return # Exit function safely

        goal = GripperCommand.Goal() # Create a new action goal message
        goal.command.position = 0.4867 if close else 0.0713 # 0.4867 = Closed | 0.0713 = Open (From physical testing!)
        
        self.get_logger().info(f'Sending Gripper Goal: {"CLOSE" if close else "OPEN"}')

        # Send goal asynchronously without blocking program
        self.gripper_client.send_goal_async(goal)

    def _reset_state(self):
        # =====================================================
        # RESET TELEOP CONTROL STATE
        #
        # Called when:
        # - Right fist opens
        # - Hand disappears
        # - Motion mode becomes inactive
        #
        # Purpose:
        # Stop robot motion immediately and clear
        # previously stored tracking information
        # =====================================================

        # Clear previous right-hand wrist position. Prevents old wrist coordinates from being
        # reused when the hand appears again
        self.prev_wrist = None
        self.rot_init   = False

        # Reset all smoothed robot velocities. Prevents robot drift after movement stops
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0
        self.current_wz = 0.0

    def _draw_overlay(self, frame, right, is_moving, twist, w, h):


        # =====================================================
        # CAMERA USER INTERFACE OVERLAY
        #
        # Draws live teleoperation information:
        #
        # - Robot state
        # - Gesture state
        # - Velocity values
        # - End-effector position
        # - User instructions
        #
        # =====================================================
        if is_moving:
            cv2.rectangle(frame, (0, 0), (w, 25), (0, 150, 0), -1)
            cv2.putText(frame, 'MOVING — Transmitting Twist',
                        (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        else:
            cv2.rectangle(frame, (0, 0), (w, 25), (0, 0, 150), -1)
            cv2.putText(frame, 'STOPPED — Close right fist to move',
                        (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        if right.detected:
            col  = (0, 255, 0) if is_moving else (0, 0, 255)
            stat = 'FIST (ACTIVE)' if right.fist_closed else 'OPEN (WAITING)'
            
            vel_txt = (f'Vel X: {twist.linear.x:.2f} | Vel Y: {twist.linear.y:.2f} | Vel Z: {twist.linear.z:.2f}')
            pos_txt = (f'EE Pos -> Px: {self.end_effector_x:.3f} | Py: {self.end_effector_y:.3f} | Pz: {self.end_effector_z:.3f}')

            cv2.putText(frame, f'RIGHT: {stat}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
            cv2.putText(frame, vel_txt, (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
            cv2.putText(frame, pos_txt, (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1) 
        else:
            cv2.putText(frame, 'RIGHT: Not Detected', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 2)

        cv2.putText(frame, 'Right Fist=MOVE | Left Pinch=GRIP | Q=Quit',
                    (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    def destroy_node(self):
        self.twist_pub.publish(Twist())
        self.tracker.destroy()
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TwistTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()




