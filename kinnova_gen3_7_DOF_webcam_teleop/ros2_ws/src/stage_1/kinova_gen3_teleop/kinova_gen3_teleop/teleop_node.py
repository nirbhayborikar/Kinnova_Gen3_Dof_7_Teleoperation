#!/usr/bin/env python3
"""
Hand Pose Publisher Node for Kinova Gen3 7-DOF.

Controls:
  LEFT HAND  → Start/Stop toggle
    - Fist closed = ARM ENABLED (right hand can control the arm)
    - Open hand / fingers spread = ARM DISABLED (arm stops, ignores right hand)
    - Even if right hand is visible, arm won't move until left hand fist starts it

  RIGHT HAND → Controls the arm (only when enabled by left hand)
    - Hand movement → EE movement (2D + 1D rotation)
    - Pinch → gripper close
    - Open hand → gripper open

Published topics:
  /target_pose          (PoseStamped) — arm EE target
  /gripper_control      (Bool)        — gripper open/close
"""

import rclpy
from rclpy.node import Node

import cv2
import mediapipe as mp
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import math

from .hand_tracker import HandTracker, HandState

# robotiq_gripper_controlleris a GripperActionController it uses an action interface, not a topic. 
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils


class HandPosePublisher(Node):
    def __init__(self):
        super().__init__('hand_pose_publisher')

        # ---- Parameters ----
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('show_camera', True)
        self.declare_parameter('pinch_threshold', 0.07)
        self.declare_parameter('alpha', 0.4)
        self.declare_parameter('dead_zone', 0.005)
        self.declare_parameter('max_vel', 0.08)
        self.declare_parameter('robot_x_min', 0.2)
        self.declare_parameter('robot_x_max', 0.6)
        self.declare_parameter('robot_y_min', -0.3)
        self.declare_parameter('robot_y_max', 0.3)
        self.declare_parameter('robot_z_min', 0.1)
        self.declare_parameter('robot_z_max', 0.6)
        self.declare_parameter('max_rotation', 1.57)

        cam_id = self.get_parameter('camera_id').value
        self.rate = self.get_parameter('rate').value
        self.show_camera = self.get_parameter('show_camera').value
        self.pinch_threshold = self.get_parameter('pinch_threshold').value
        self.alpha = self.get_parameter('alpha').value
        self.dead_zone = self.get_parameter('dead_zone').value
        self.max_vel = self.get_parameter('max_vel').value
        self.robot_x_min = self.get_parameter('robot_x_min').value
        self.robot_x_max = self.get_parameter('robot_x_max').value
        self.robot_y_min = self.get_parameter('robot_y_min').value
        self.robot_y_max = self.get_parameter('robot_y_max').value
        self.robot_z_min = self.get_parameter('robot_z_min').value
        self.robot_z_max = self.get_parameter('robot_z_max').value
        self.max_rotation = self.get_parameter('max_rotation').value

        # ---- Publishers (single arm only) ----
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        
        #-------------Publisher gripper-----------------
        self.gripper_pub = self.create_publisher(Bool, '/gripper_control', 10)




        # ---- Camera ----
        self.cap = cv2.VideoCapture(cam_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera {cam_id}')
            raise RuntimeError(f'Cannot open camera {cam_id}')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # ---- Hand Tracker ----
        self.tracker = HandTracker(pinch_threshold=self.pinch_threshold)

        # ---- Smoothing state (right hand only) ----
        self.smooth = {'x': 0.5, 'y': 0.5, 'rot': 0.0, 'init': False}

        # ---- ARM ENABLED STATE (controlled by left hand) ----
        self.arm_enabled = False  # Starts DISABLED — must close left fist to start

        # ---- Hand tracking state ----
        self.left_active = False
        self.right_active = False
        self.hand_timeout = 2.0
        self.left_last_seen = 0.0
        self.right_last_seen = 0.0

        self.frame_count = 0

        # ---- Timer ----
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        self.get_logger().info('========================================')
        self.get_logger().info('  Kinova Gen3 Teleop Started')
        self.get_logger().info('  LEFT hand:  Fist=START, Open=STOP')
        self.get_logger().info('  RIGHT hand: Move arm + Pinch=Grip')
        self.get_logger().info('  Arm starts DISABLED — close left fist!')
        self.get_logger().info('  Press Q in camera window to quit')
        self.get_logger().info('========================================')

    def _smooth_update(self, raw_x, raw_y, raw_rot):
        """EMA smoothing with dead-zone and velocity clamping."""
        s = self.smooth
        if not s['init']:
            s['x'], s['y'], s['rot'] = raw_x, raw_y, raw_rot
            s['init'] = True
            return raw_x, raw_y, raw_rot

        dx = raw_x - s['x']
        dy = raw_y - s['y']
        dr = raw_rot - s['rot']
        while dr > math.pi: dr -= 2 * math.pi
        while dr < -math.pi: dr += 2 * math.pi

        if abs(dx) < self.dead_zone: dx = 0.0
        if abs(dy) < self.dead_zone: dy = 0.0
        if abs(dr) < self.dead_zone * 5: dr = 0.0

        dx = np.clip(dx, -self.max_vel, self.max_vel)
        dy = np.clip(dy, -self.max_vel, self.max_vel)
        dr = np.clip(dr, -0.15, 0.15)

        s['x'] += self.alpha * dx
        s['y'] += self.alpha * dy
        s['rot'] += self.alpha * dr

        return s['x'], s['y'], s['rot']

    def _map_to_pose(self, cx, cy, roll):
        """Map normalized hand coords to robot PoseStamped."""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'base_link' # confirm with tf: ros2 run tf2_ros tf2_echo world base_link
        # worls say in valid and then base_link will start with values

        # Single arm — no Y offset needed
        robot_x = (self.robot_x_min + self.robot_x_max) / 2.0
        robot_y = self.robot_y_min + cx * (self.robot_y_max - self.robot_y_min)
        robot_z = self.robot_z_min + (1.0 - cy) * (self.robot_z_max - self.robot_z_min)

        pose.pose.position.x = robot_x
        pose.pose.position.y = robot_y
        pose.pose.position.z = robot_z

        # Orientation: EE pointing down, rotated by hand roll
        clamped_roll = np.clip(roll, -self.max_rotation, self.max_rotation)
        cx_q = math.cos(math.pi / 2)
        sx_q = math.sin(math.pi / 2)
        cz_q = math.cos(clamped_roll / 2)
        sz_q = math.sin(clamped_roll / 2)

        pose.pose.orientation.x = sx_q * cz_q
        pose.pose.orientation.y = sx_q * sz_q
        pose.pose.orientation.z = cx_q * sz_q
        pose.pose.orientation.w = cx_q * cz_q

        return pose

    def timer_callback(self):
        """Main loop: capture → track → check start/stop → smooth → publish."""
        ret, frame = self.cap.read()
        if not ret:
            return

        left, right, annotated = self.tracker.process_frame(frame)

        self.frame_count += 1
        current_time = self.get_clock().now().nanoseconds / 1e9
        h, w = annotated.shape[:2]

        # ===== LEFT HAND: START/STOP TOGGLE =====
        if left.detected:
            self.left_last_seen = current_time
            if not self.left_active:
                self.get_logger().info('Left hand detected')
            self.left_active = True

            if left.fist_closed:
                if not self.arm_enabled:
                    self.get_logger().info('>>> ARM ENABLED (left fist closed)')
                self.arm_enabled = True
            else:
                if self.arm_enabled:
                    self.get_logger().info('>>> ARM DISABLED (left hand open)')
                self.arm_enabled = False
        else:
            if self.left_active:
                if current_time - self.left_last_seen > self.hand_timeout:
                    self.left_active = False
                    self.get_logger().info('Left hand lost')
                    # Don't change arm_enabled — keep last state

        # ===== RIGHT HAND: ARM CONTROL (only when enabled) =====
        if right.detected:
            self.right_last_seen = current_time
            if not self.right_active:
                self.get_logger().info('Right hand detected')
            self.right_active = True

            if self.arm_enabled:
                # Smooth and map
                sx, sy, srot = self._smooth_update(
                    right.wrist_x, right.wrist_y, right.rotation)
                pose = self._map_to_pose(sx, sy, srot)

                # Gripper
                gripper_msg = Bool()
                gripper_msg.data = not right.gripper_closed  # True=open, False=close

                # Publish
                self.pose_pub.publish(pose)
                self.gripper_pub.publish(gripper_msg)
        else:
            if self.right_active:
                if current_time - self.right_last_seen > self.hand_timeout:
                    self.right_active = False
                    self.smooth['init'] = False
                    self.get_logger().info('Right hand lost')

        # ===== DRAW OVERLAY =====
        self._draw_overlay(annotated, left, right, w, h)

        if self.show_camera:
            cv2.imshow('Kinova Gen3 Teleop', annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

    def _draw_overlay(self, frame, left, right, w, h):
        """Draw status info on camera feed."""

        # Arm status banner
        if self.arm_enabled:
            cv2.rectangle(frame, (0, 0), (w, 25), (0, 100, 0), -1)
            cv2.putText(frame, 'ARM ENABLED — Right hand controls arm',
                        (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        else:
            cv2.rectangle(frame, (0, 0), (w, 25), (0, 0, 150), -1)
            cv2.putText(frame, 'ARM DISABLED — Close left fist to start',
                        (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        # Left hand status
        if left.detected:
            color_l = (0, 255, 255)  # Yellow
            fist_txt = 'FIST (START)' if left.fist_closed else 'OPEN (STOP)'
            cv2.putText(frame, f'LEFT: {fist_txt}', (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_l, 2)
        else:
            cv2.putText(frame, 'LEFT: ---', (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 2)

        # Right hand status
        if right.detected:
            color_r = (0, 255, 0) if self.arm_enabled else (0, 0, 255)
            grip_txt = 'GRIP' if right.gripper_closed else 'OPEN'
            status = 'ACTIVE' if self.arm_enabled else 'WAITING'
            cv2.putText(frame, f'RIGHT: {status} [{grip_txt}]', (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_r, 2)
        else:
            cv2.putText(frame, 'RIGHT: ---', (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 2)

        # Instructions
        cv2.putText(frame, 'Left fist=START | Left open=STOP | Right pinch=Grip | Q=Quit',
                    (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    def destroy_node(self):
        self.tracker.destroy()
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
