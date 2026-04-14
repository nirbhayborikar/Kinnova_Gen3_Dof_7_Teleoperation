#!/usr/bin/env python3
"""
Velocity-Based Teleop Node for Kinova Gen3 7-DOF.
Targets: picknik_twist_controller/PicknikTwistController

Controls:
  RIGHT HAND ONLY
    - Open hand   = WAITING / STOP (Publishes zero velocity)
    - Closed fist = MOVING (Hand delta maps to Cartesian Twist velocity)
"""




"""
    # Left 
    open gripper close gripperst 

    close fist gripper close 
    open fist gripper open

    in close fist left  left right hand movement -- move arm up down

    in close fitst left , up down hand movement -- move arm forward backward



    # right 

    open fist deactivated
    close fist activate
    close fist one place left arm has control

    
    close fist up downn movement -- arm left right in y direction
    close fist left right ---- arm up down 
    
     """



import math
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

# marker and points
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# to log the x,y,z position of end-effector use TF2
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

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
        
        self.declare_parameter('vel_scale_y',      5.0) # Lateral (left/right)
        self.declare_parameter('vel_scale_z',      5.0) # Vertical (up/down)
        self.declare_parameter('vel_scale_roll',   5.0) # Wrist rotation
        
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
        self.v_scale_roll = self.get_parameter('vel_scale_roll').value 
        self.max_lin      = self.get_parameter('max_linear_vel').value
        self.max_ang      = self.get_parameter('max_angular_vel').value

        # ---- Publishers ----
        self.twist_pub = self.create_publisher(Twist, '/twist_controller/commands', 10)
        self.arrow_pub = self.create_publisher(Marker, '/teleop_velocity_arrow', 10)

        # ---- TF2 Listener ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        # ---- Gripper Client ----
        self.gripper_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')
        self.last_gripper_state = None 

        # ---- Delta control state ----
        self.prev_wrist = None
        self.smooth_rot = 0.0
        self.rot_init   = False
        self.prev_left_wrist = None 
        
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0
        self.current_wz = 0.0

        # ---- Timer ----
        self.timer = self.create_timer(1.0 / self.rate, self._timer_cb)

        self.get_logger().info('========================================')
        self.get_logger().info('  Kinova Twist Teleop (Velocity Mode)')
        self.get_logger().info('  RIGHT HAND: Drive Robot (X, Z, Roll)')
        self.get_logger().info('  LEFT HAND:  Gripper & Strafe (Y)')
        self.get_logger().info('========================================')

    def _timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        left, right, annotated = self.tracker.process_frame(frame)
        h, w = annotated.shape[:2]

        twist_msg = Twist()
        is_moving = False

        # =========================================================
        # 1. GRIPPER CONTROL
        # =========================================================
        if right.detected and right.fist_closed and left.detected:
            should_close = left.fist_closed or left.gripper_closed 
            if should_close != self.last_gripper_state:
                self._send_gripper_command(close=should_close)
                self.last_gripper_state = should_close
            
        # =========================================================
        # 2. CALCULATE LEFT HAND MOVEMENT
        # =========================================================
        cam_dx_left = 0.0
        cam_dy_left = 0.0

        if left.detected and left.fist_closed:
            if self.prev_left_wrist is None:
                self.prev_left_wrist = (left.wrist_x, left.wrist_y)
            else:
                raw_dx = left.wrist_x - self.prev_left_wrist[0]
                raw_dy = left.wrist_y - self.prev_left_wrist[1]
                
                if abs(raw_dx) >= self.dead_zone: cam_dx_left = raw_dx
                if abs(raw_dy) >= self.dead_zone: cam_dy_left = raw_dy
                
                self.prev_left_wrist = (left.wrist_x, left.wrist_y)
        else:
            self.prev_left_wrist = None

        # =========================================================
        # 3. CALCULATE RIGHT HAND & APPLY "SMART SWITCH"
        # =========================================================
        if right.detected and right.fist_closed:
            is_moving = True
            if self.prev_wrist is None:
                self.prev_wrist = (right.wrist_x, right.wrist_y)
            else:
                raw_dx_right = right.wrist_x - self.prev_wrist[0]
                raw_dy_right = right.wrist_y - self.prev_wrist[1]
                cam_dx_right = raw_dx_right if abs(raw_dx_right) >= self.dead_zone else 0.0
                cam_dy_right = raw_dy_right if abs(raw_dy_right) >= self.dead_zone else 0.0

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
                
                twist_msg.linear.x = np.clip(self.current_vx, -self.max_lin, self.max_lin)
                twist_msg.linear.y = np.clip(self.current_vy, -self.max_lin, self.max_lin)
                twist_msg.linear.z = np.clip(self.current_vz, -self.max_lin, self.max_lin)
                
                self.prev_wrist = (right.wrist_x, right.wrist_y)
            
            # --- Smooth rotation (Wrist Roll) ---
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
        self.twist_pub.publish(twist_msg)

        # ---- TF2 Update ----
        try:
            t = self.tf_buffer.lookup_transform('base_link', 'grasping_frame', rclpy.time.Time())
            self.end_effector_x = t.transform.translation.x
            self.end_effector_y = t.transform.translation.y
            self.end_effector_z = t.transform.translation.z
        except TransformException:
            pass

        # ── Overlay + display ────────────────────────────────────────
        self._draw_overlay(annotated, right, is_moving, twist_msg, w, h)

        if self.show_camera:
            cv2.imshow('Kinova Twist Teleop', annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

    def _send_gripper_command(self, close=True):
        if not self.gripper_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn('Gripper action server not ready')
            return

        goal = GripperCommand.Goal()
        goal.command.position = 0.4867 if close else 0.0713
        
        self.get_logger().info(f'Sending Gripper Goal: {"CLOSE" if close else "OPEN"}')
        self.gripper_client.send_goal_async(goal)

    def _reset_state(self):
        self.prev_wrist = None
        self.rot_init   = False
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0
        self.current_wz = 0.0

    def _draw_overlay(self, frame, right, is_moving, twist, w, h):
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



"""
    # Left 
    open gripper close gripperst 

    close fist gripper close 
    open fist gripper open

    in close fist left  left right hand movement -- move arm up down

    in close fitst left , up down hand movement -- move arm forward backward



    # right 

    open fist deactivated
    close fist activate
    close fist one place left arm has control

    
    close fist up downn movement -- arm left right in y direction
    close fist left right ---- arm up down 
    
     """

