#!/usr/bin/env python3
"""
Velocity-Based Teleop Node for Kinova Gen3 7-DOF.
Targets: picknik_twist_controller/PicknikTwistController

Controls:
  RIGHT HAND ONLY
    - Open hand   = WAITING / STOP (Publishes zero velocity)
    - Closed fist = MOVING (Hand delta maps to Cartesian Twist velocity)
"""

import math
#import sys
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist



# marker and points

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# to log the x,y,z position of end-effector use TF2
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


# # ── Force real pip mediapipe from venv ──────────────────────────────
# _REAL_MP = '/opt/venv/lib/python3.12/site-packages'
# if _REAL_MP not in sys.path:
#     sys.path.insert(0, _REAL_MP)
# for _k in list(sys.modules.keys()):
#     if _k == 'mediapipe' or _k.startswith('mediapipe.'):
#         del sys.modules(k)

# up down it going forwa rd and , left righ going in z axis.

import mediapipe as mp
from .hand_tracker import HandTracker

mp_hands   = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils


class TwistTeleopNode(Node):
    def __init__(self):
        super().__init__('twist_teleop_publisher')

        # ---- Parameters ----
        self.declare_parameter('camera_id',        0)
        self.declare_parameter('rate',             30.0)
        self.declare_parameter('show_camera',      True)
        
        # Smoothing and Deadzones
        self.declare_parameter('alpha',            0.4)
        self.declare_parameter('dead_zone',        0.002) 
        
        # Velocity Scaling (How aggressive the arm moves based on hand speed)
        # Note: Twist expects meters/sec. Hand deltas are normalized screen pixels (0.0 to 1.0).
        self.declare_parameter('vel_scale_y',      5.0) # Lateral (left/right) #1.5
        self.declare_parameter('vel_scale_z',      5.0) # Vertical (up/down) #1.5
        self.declare_parameter('vel_scale_roll',   5.0) # Wrist rotation
        
        # Max velocity limits for safety (meters/sec & rad/sec)
        self.declare_parameter('max_linear_vel',   4.0)
        self.declare_parameter('max_angular_vel',  2.0)

        # Load parameters
        cam_id            = self.get_parameter('camera_id').value
        self.rate         = self.get_parameter('rate').value
        self.show_camera  = self.get_parameter('show_camera').value
        self.alpha        = self.get_parameter('alpha').value
        self.dead_zone    = self.get_parameter('dead_zone').value
        self.v_scale_y    = self.get_parameter('vel_scale_y').value
        self.v_scale_z    = self.get_parameter('vel_scale_z').value
        self.v_scale_roll = self.get_parameter('vel_scale_roll').value # roll velocity or rotation velocity scaling parameter 
        self.max_lin      = self.get_parameter('max_linear_vel').value
        self.max_ang      = self.get_parameter('max_angular_vel').value

        # ---- Publishers ----
        # Publishing Twist commands for the PicknikTwistController
        self.twist_pub = self.create_publisher(Twist, '/twist_controller/commands', 10)



        # visulaization marker
        self.arrow_pub = self.create_publisher(Marker, '/teleop_velocity_arrow', 10)

        # ---- TF2 Listener (To get real robot position) ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Variables to store the current position
        self.end_effector_x = 0.0
        self.end_effector_y = 0.0
        self.end_effector_z = 0.0


        # ---- Camera ----
        self.cap = cv2.VideoCapture(cam_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera {cam_id}')
            raise RuntimeError(f'Cannot open camera {cam_id}')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # ---- Hand tracker ----
        self.tracker = HandTracker(pinch_threshold=0.07)

        # ---- Delta control state ----
        self.prev_wrist = None
        self.smooth_rot = 0.0
        self.rot_init   = False
        
        # Filtered velocity state (for EMA)
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0
        self.current_wz = 0.0

        # ---- Timer ----
        self.timer = self.create_timer(1.0 / self.rate, self._timer_cb)

        self.get_logger().info('========================================')
        self.get_logger().info('  Kinova Twist Teleop (Velocity Mode)')
        self.get_logger().info('  RIGHT HAND ONLY:')
        self.get_logger().info('    Open Hand   = WAIT / STOP')
        self.get_logger().info('    Closed Fist = MOVE ROBOT')
        self.get_logger().info('  Press Q to quit')
        self.get_logger().info('========================================')

    def _timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # Process frame (ignoring left hand completely)
        _, right, annotated = self.tracker.process_frame(frame)
        h, w = annotated.shape[:2]

        twist_msg = Twist()
        is_moving = False

        if right.detected:
            # The clutch: only move if fist is closed
            if right.fist_closed:
                is_moving = True
                
                if self.prev_wrist is None:
                    # First frame of closing fist — set anchor, don't move yet
                    self.prev_wrist = (right.wrist_x, right.wrist_y) # so when reset we goes here again and then takes this 
                else:
                    # Calculate raw pixel deltas
                    raw_dy = right.wrist_x - self.prev_wrist[0] # Camera X controls Robot Y (lateral)
                    raw_dz = right.wrist_y - self.prev_wrist[1] # Camera Y controls Robot Z (vertical)

                    # Apply dead zone to kill webcam jitter
                    if abs(raw_dy) < self.dead_zone: raw_dy = 0.0
                    if abs(raw_dz) < self.dead_zone: raw_dz = 0.0

                    # Map to target velocities
                    # Note: Camera Y is inverted (0 is top, 1 is bottom). 
                    # If hand moves up (negative raw_dz), robot should move up (positive Z vel).
                    target_vy = -raw_dy * self.v_scale_y
                    target_vz = -raw_dz * self.v_scale_z

                    # Smooth velocities using EMA
                    self.current_vy = (self.alpha * target_vy) + ((1 - self.alpha) * self.current_vy)
                    self.current_vz = (self.alpha * target_vz) + ((1 - self.alpha) * self.current_vz)

                    # Clamp to safe physical limits
                    twist_msg.linear.y = np.clip(self.current_vy, -self.max_lin, self.max_lin)
                    twist_msg.linear.z = np.clip(self.current_vz, -self.max_lin, self.max_lin)
                    twist_msg.linear.x = 0.0 # Depth fixed in 2D webcam setup

                    # Update anchor for next frame
                    self.prev_wrist = (right.wrist_x, right.wrist_y)

                # ---- Smooth rotation (Wrist Roll) ----
                raw_rot = right.rotation
                if not self.rot_init:
                    self.smooth_rot = raw_rot
                    self.rot_init   = True
                else:
                    # Calculate rotational delta
                    dr = raw_rot - self.smooth_rot
                    while dr >  math.pi: dr -= 2 * math.pi
                    while dr < -math.pi: dr += 2 * math.pi
                    
                    target_wz = dr * self.v_scale_roll # roll scaling parameter 
                    self.current_wz = (self.alpha * target_wz) + ((1 - self.alpha) * self.current_wz)
                    
                    twist_msg.angular.z = np.clip(self.current_wz, -self.max_ang, self.max_ang)
                    self.smooth_rot = raw_rot # update anchor

            else:
                # Hand is OPEN -> Stop moving immediately
                self._reset_state()
        else:
            # Hand LOST -> Stop moving immediately
            self._reset_state()



        # befor publishing we should have the marker for visualization


        # ---------------------------------------------------------
        #  Velocity Arrow Visualization (RViz)
        # ---------------------------------------------------------
        arrow = Marker()
        arrow.header.frame_id = 'base_link'  # Matches the twist command frame
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.ns = 'teleop_direction'
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD

        # Where the arrow starts. 
        # (Putting it at Z=0.5 makes it float like a compass right above the robot base)
        start_pt = Point(x=0.0, y=0.0, z=0.5) 

        # Where the arrow ends (Start Point + Velocity Vector)
        # We multiply by 2.0 just to make the arrow visually longer in RViz
        visual_scale = 2.0 
        end_pt = Point()
        end_pt.x = start_pt.x + (twist_msg.linear.x * visual_scale)
        end_pt.y = start_pt.y + (twist_msg.linear.y * visual_scale)
        end_pt.z = start_pt.z + (twist_msg.linear.z * visual_scale)

        arrow.points = [start_pt, end_pt]

        # Arrow Dimensions (shaft diameter, head diameter, head length)
        arrow.scale.x = 0.02
        arrow.scale.y = 0.04
        arrow.scale.z = 0.04

        # Color: Bright Cyan (so it stands out against the robot)
        arrow.color.r = 0.0
        arrow.color.g = 1.0
        arrow.color.b = 1.0
        # If moving, make it solid. If stopped, make it disappear (alpha = 0)
        arrow.color.a = 0.8 if is_moving else 0.0 

        # Publish the arrow!
        self.arrow_pub.publish(arrow)
        # ---------------------------------------------------------


        # Publish the Twist command (Always publish to keep controller alive/zeroed)
        self.twist_pub.publish(twist_msg)




        ## add the 
        # ---------------------------------------------------------
        # Get Real Robot Position via TF2
        # ---------------------------------------------------------
        try:
            # Look up the transform from base_link to grasping_frame
            t = self.tf_buffer.lookup_transform(
                'base_link', 
                'grasping_frame', # Change to 'end_effector_link' if needed
                rclpy.time.Time())
            
            # Extract the translation (X, Y, Z in meters)
            self.end_efffector_x = t.transform.translation.x
            self.end_efffector_y = t.transform.translation.y
            self.end_efffector_z = t.transform.translation.z
            
            # Optional: Log it to the terminal occasionally
            self.get_logger().info(f"EE Pos -> X:{self.end_efffector_x:.3f} Y:{self.end_efffector_y:.3f} Z:{self.end_efffector_z:.3f}")
            
        except TransformException as ex:
            # If the robot isn't running yet or TF is missing, just pass safely
            self.get_logger().info('The robot tf is not available yet')




        # ── Overlay + display ────────────────────────────────────────
        self._draw_overlay(annotated, right, is_moving, twist_msg, w, h)

        if self.show_camera:
            cv2.imshow('Kinova Twist Teleop', annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

    def _reset_state(self):
        """Resets anchors and velocity history to ensure a clean stop."""
        self.prev_wrist = None
        self.rot_init   = False
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0
        self.current_wz = 0.0

    def _draw_overlay(self, frame, right, is_moving, twist, w, h):
        # Status banner
        if is_moving:
            cv2.rectangle(frame, (0, 0), (w, 25), (0, 150, 0), -1)
            cv2.putText(frame, 'MOVING — Transmitting Twist',
                        (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        else:
            cv2.rectangle(frame, (0, 0), (w, 25), (0, 0, 150), -1)
            cv2.putText(frame, 'STOPPED — Close right fist to move',
                        (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        # Right hand UI
        if right.detected:
            col  = (0, 255, 0) if is_moving else (0, 0, 255)
            stat = 'FIST (ACTIVE)' if right.fist_closed else 'OPEN (WAITING)'
            
            vel_txt = (f'Vel Y: {twist.linear.y:.3f} m/s | '
                       f'Vel Z: {twist.linear.z:.3f} m/s | '
                       f'Rot Z: {twist.angular.z:.3f} rad/s')

            cv2.putText(frame, f'RIGHT: {stat}', (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
            cv2.putText(frame, vel_txt, (10, 85),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
        else:
            cv2.putText(frame, 'RIGHT: Not Detected', (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 2)

        # Footer
        cv2.putText(frame,
                    'Right Fist=MOVE | Right Open=STOP | Q=Quit',
                    (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    def destroy_node(self):
        # Send one final zero twist to guarantee robot stops on shutdown
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



######## if tf2 needed to use in video overlay then use

# def _draw_overlay(self, frame, right, is_moving, twist, w, h):
#         # [ ... Keep your existing Status Banner code here ... ]

#         # Right hand UI
#         if right.detected:
#             col  = (0, 255, 0) if is_moving else (0, 0, 255)
#             stat = 'FIST (ACTIVE)' if right.fist_closed else 'OPEN (WAITING)'
            
#             # The Velocity we are sending
#             vel_txt = (f'Vel Y: {twist.linear.y:.3f} | Vel Z: {twist.linear.z:.3f}')
            
#             # The actual physical position of the robot
#             pos_txt = (f'EE Pos -> Px: {self.robot_x:.3f} | Py: {self.robot_y:.3f} | Pz: {self.robot_z:.3f}')

#             cv2.putText(frame, f'RIGHT: {stat}', (10, 60),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
#             cv2.putText(frame, vel_txt, (10, 85),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
#             # Add the new position text in Cyan
#             cv2.putText(frame, pos_txt, (10, 110),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1) 
#         else:
#             cv2.putText(frame, 'RIGHT: Not Detected', (10, 60),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 2)

#         # [ ... Keep your existing Footer code here ... ]