#!/usr/bin/env python3
"""
Hand Pose Publisher Node for Kinova Gen3 7-DOF.

Controls:
  LEFT HAND  → Start/Stop toggle
    - Fist closed  = ARM ENABLED
    - Open hand    = ARM DISABLED

  RIGHT HAND → Controls the arm (only when enabled)
    - Hand movement → EE delta movement (incremental, not absolute)
    - Pinch         → gripper close
    - Open hand     → gripper open

Published topics:
  /target_pose      (PoseStamped) — arm EE target  (delta-based)
  /gripper_control  (Bool)        — gripper open/close (on change only)
"""

import math
import rclpy
from rclpy.node import Node
import cv2
import sys


# ── Force real pip mediapipe from venv ──────────────────────────────
_REAL_MP = '/opt/venv/lib/python3.12/site-packages'
if _REAL_MP not in sys.path:
    sys.path.insert(0, _REAL_MP)
for _k in list(sys.modules.keys()):
    if _k == 'mediapipe' or _k.startswith('mediapipe.'):
        del sys.modules[_k]
import mediapipe as mp

import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from .hand_tracker import HandTracker, HandState

mp_hands    = mp.solutions.hands
mp_drawing  = mp.solutions.drawing_utils


class HandPosePublisher(Node):

    def __init__(self):
        super().__init__('hand_pose_publisher')

        # ---- Parameters ----
        self.declare_parameter('camera_id',        0)
        self.declare_parameter('rate',             30.0)
        self.declare_parameter('show_camera',      True)
        self.declare_parameter('pinch_threshold',  0.07)
        self.declare_parameter('alpha',            0.4)
        self.declare_parameter('dead_zone',        0.005)
        # delta scale: how many robot-metres per normalised pixel unit per frame
        self.declare_parameter('delta_scale_xy',   0.03) #  0.003
        self.declare_parameter('delta_scale_z',    0.003)
        self.declare_parameter('robot_x_min',      0.20)
        self.declare_parameter('robot_x_max',      0.60)
        self.declare_parameter('robot_y_min',     -0.30)
        self.declare_parameter('robot_y_max',      0.30)
        self.declare_parameter('robot_z_min',      0.10)
        self.declare_parameter('robot_z_max',      0.60)
        self.declare_parameter('max_rotation',     1.57)
        # home pose the arm starts from when enabled
        self.declare_parameter('home_x',           0.5)
        self.declare_parameter('home_y',           0.5)
        self.declare_parameter('home_z',           0.0)
        self.declare_parameter('home_rot',           0.0)

        cam_id              = self.get_parameter('camera_id').value
        self.rate           = self.get_parameter('rate').value
        self.show_camera    = self.get_parameter('show_camera').value
        self.pinch_thr      = self.get_parameter('pinch_threshold').value
        self.alpha          = self.get_parameter('alpha').value
        self.dead_zone      = self.get_parameter('dead_zone').value
        self.delta_xy       = self.get_parameter('delta_scale_xy').value
        self.delta_z        = self.get_parameter('delta_scale_z').value
        self.rx_min         = self.get_parameter('robot_x_min').value
        self.rx_max         = self.get_parameter('robot_x_max').value
        self.ry_min         = self.get_parameter('robot_y_min').value
        self.ry_max         = self.get_parameter('robot_y_max').value
        self.rz_min         = self.get_parameter('robot_z_min').value
        self.rz_max         = self.get_parameter('robot_z_max').value
        self.max_rotation   = self.get_parameter('max_rotation').value

        # ---- Publishers ----
        self.pose_pub    = self.create_publisher(PoseStamped, '/target_pose',    10)
        self.gripper_pub = self.create_publisher(Bool,        '/gripper_control', 10)


        self.arm_enabled_pub = self.create_publisher(Bool,'/arm_enabled', 10) # arm enable or not publish

        # ---- Camera ----
        self.cap = cv2.VideoCapture(cam_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera {cam_id}')
            raise RuntimeError(f'Cannot open camera {cam_id}')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # ---- Hand tracker ----
        self.tracker = HandTracker(pinch_threshold=self.pinch_thr)

        # ---- Arm enable state ----
        self.arm_enabled = False

        self.last_arm_enabled = None # last arm enabled check it 

        # ---- Hand visibility tracking ----
        self.left_active      = False
        self.right_active     = False
        self.hand_timeout     = 2.0
        self.left_last_seen   = 0.0
        self.right_last_seen  = 0.0

        # ---- Delta control state ----
        # Current robot EE position (accumulated deltas)
        self.robot_pos = {
            'x': self.get_parameter('home_x').value,
            'y': self.get_parameter('home_y').value,
            'z': self.get_parameter('home_z').value,
            'rot': self.get_parameter('home_rot').value,
        }
        # Previous normalised wrist coords (set on first right-hand frame)
        self.prev_wrist = None

        # Smoothed rotation
        self.smooth_rot = 0.0
        self.rot_init   = False

        # ---- Gripper state (only publish on change) ----
        self.last_gripper_state = None

        # ---- Timer ----
        self.timer = self.create_timer(1.0 / self.rate, self._timer_cb)

        self.get_logger().info('========================================')
        self.get_logger().info('  Kinova Gen3 Teleop  (delta mode)')
        self.get_logger().info('  LEFT  fist = ARM ON  |  open = ARM OFF')
        self.get_logger().info('  RIGHT hand = move EE |  pinch = grip')
        self.get_logger().info('  Press Q to quit')
        self.get_logger().info('========================================')

    # ------------------------------------------------------------------ #
    #  Main loop                                                           #
    # ------------------------------------------------------------------ #

    def _timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        left, right, annotated = self.tracker.process_frame(frame)
        now = self.get_clock().now().nanoseconds / 1e9
        h, w = annotated.shape[:2]

        # ── LEFT HAND: enable / disable arm ──────────────────────────
        if left.detected:
            self.left_last_seen = now
            if not self.left_active:
                self.get_logger().info('Left hand detected')
            self.left_active = True

            was_enabled = self.arm_enabled
            self.arm_enabled = left.fist_closed
            if self.arm_enabled and not was_enabled:
                self.get_logger().info('>>> ARM ENABLED')
                # Reset delta origin so arm doesn't jump
                self.prev_wrist = None
            elif not self.arm_enabled and was_enabled:
                self.get_logger().info('>>> ARM DISABLED')
                self.prev_wrist = None

        elif self.left_active and (now - self.left_last_seen > self.hand_timeout):
            self.left_active = False
            self.get_logger().info('Left hand lost')

        # Publish arm_enabled state on change
        if self.arm_enabled != self.last_arm_enabled:
            msg = Bool()
            msg.data = self.arm_enabled
            self.arm_enabled_pub.publish(msg)
            self.last_arm_enabled = self.arm_enabled
            self.get_logger().info(f'Arm enabled: {self.arm_enabled}')



        

        # ── RIGHT HAND: move arm + gripper ───────────────────────────
        if right.detected:
            self.right_last_seen = now
            if not self.right_active:
                self.get_logger().info('Right hand detected')
            self.right_active = True

            if self.arm_enabled:
                # ---- Delta position ----
                if self.prev_wrist is None:
                    # First frame after enable — anchor, don't move
                    self.prev_wrist = (right.wrist_x, right.wrist_y)
                else:
                    raw_dx = right.wrist_x - self.prev_wrist[0]
                    raw_dy = right.wrist_y - self.prev_wrist[1]

                    # Dead zone
                    if abs(raw_dx) < self.dead_zone: raw_dx = 0.0
                    if abs(raw_dy) < self.dead_zone: raw_dy = 0.0

                    # EMA smoothing on delta
                    dx = self.alpha * raw_dx
                    dy = self.alpha * raw_dy

                    # Map: image-right → robot-Y-negative
                    #      image-up    → robot-Z-positive  (image y is inverted)
                    self.robot_pos['y'] = np.clip(
                        self.robot_pos['y'] - dx * self.delta_xy / self.alpha,
                        self.ry_min, self.ry_max)
                    self.robot_pos['z'] = np.clip(
                        self.robot_pos['z'] - dy * self.delta_z / self.alpha,
                        self.rz_min, self.rz_max)
                    # X (depth) is fixed — MediaPipe can't give true depth
                    # Uncomment & map a second hand or slider for X control

                    self.prev_wrist = (right.wrist_x, right.wrist_y)

                # ---- Smooth rotation ----
                raw_rot = right.rotation
                if not self.rot_init:
                    self.smooth_rot = raw_rot
                    self.rot_init   = True
                else:
                    dr = raw_rot - self.smooth_rot
                    while dr >  math.pi: dr -= 2 * math.pi
                    while dr < -math.pi: dr += 2 * math.pi
                    self.smooth_rot += self.alpha * np.clip(dr, -0.15, 0.15)

                # ---- Publish pose ----
                pose = self._build_pose(
                    self.robot_pos['x'],
                    self.robot_pos['y'],
                    self.robot_pos['z'],
                    self.smooth_rot)
                self.pose_pub.publish(pose)

            # ---- Gripper (publish only on state change) ----
            gripper_open = not right.gripper_closed
            if gripper_open != self.last_gripper_state:
                msg = Bool()
                msg.data = gripper_open
                self.gripper_pub.publish(msg)
                self.last_gripper_state = gripper_open
                self.get_logger().info(
                    f'Gripper Change: {"OPEN" if gripper_open else "CLOSE"}')

        elif self.right_active and (now - self.right_last_seen > self.hand_timeout):
            self.right_active = False
            self.prev_wrist   = None
            self.rot_init     = False
            self.get_logger().info('Right hand lost')

        # ── Overlay + display ────────────────────────────────────────
        self._draw_overlay(annotated, left, right, w, h)
        if self.show_camera:
            cv2.imshow('Kinova Gen3 Teleop', annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

    # ------------------------------------------------------------------ #
    #  Helpers                                                             #
    # ------------------------------------------------------------------ #

    def _build_pose(self, x, y, z, roll):
        """Build a PoseStamped from robot-frame XYZ + roll."""
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = 'base_link'

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)

        # EE pointing down, rotated by hand roll
        clamped = np.clip(roll, -self.max_rotation, self.max_rotation)
        cx_q = math.cos(math.pi / 4)   # 90° around X  →  cos/sin of half-angle
        sx_q = math.sin(math.pi / 4)
        cz_q = math.cos(clamped / 2)
        sz_q = math.sin(clamped / 2)

        pose.pose.orientation.x = sx_q * cz_q
        pose.pose.orientation.y = sx_q * sz_q
        pose.pose.orientation.z = cx_q * sz_q
        pose.pose.orientation.w = cx_q * cz_q

        return pose

    def _draw_overlay(self, frame, left, right, w, h):
        # Status banner
        if self.arm_enabled:
            cv2.rectangle(frame, (0, 0), (w, 25), (0, 100, 0), -1)
            cv2.putText(frame, 'ARM ENABLED — right hand controls EE',
                        (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        else:
            cv2.rectangle(frame, (0, 0), (w, 25), (0, 0, 150), -1)
            cv2.putText(frame, 'ARM DISABLED — close left fist to start',
                        (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        # Left hand
        if left.detected:
            txt = 'FIST (ON)' if left.fist_closed else 'OPEN (OFF)'
            cv2.putText(frame, f'LEFT: {txt}', (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            cv2.putText(frame, 'LEFT: ---', (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 2)

        # Right hand
        if right.detected:
            col  = (0, 255, 0) if self.arm_enabled else (0, 0, 255)
            grip = 'GRIP' if right.gripper_closed else 'OPEN'
            stat = 'ACTIVE' if self.arm_enabled else 'WAITING'
            # Show current robot target position
            pos_txt = (f'EE x={self.robot_pos["x"]:.2f} '
                       f'y={self.robot_pos["y"]:.2f} '
                       f'z={self.robot_pos["z"]:.2f}')
            cv2.putText(frame, f'RIGHT: {stat} [{grip}]', (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
            cv2.putText(frame, pos_txt, (10, 105),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
        else:
            cv2.putText(frame, 'RIGHT: ---', (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 2)

        cv2.putText(frame,
                    'Left fist=ON | Left open=OFF | Right pinch=Grip | Q=Quit',
                    (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    # ------------------------------------------------------------------ #
    #  Cleanup                                                             #
    # ------------------------------------------------------------------ #

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