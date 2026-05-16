#!/usr/bin/env python3
"""
Kinova Gen3 MoveIt Servo Teleop via MediaPipe hand tracking.

Replaces BOTH hand_pose_publisher + arm_controller.
Publishes TwistStamped velocity commands directly to MoveIt Servo.
No /compute_ik needed — Servo handles IK internally.

Controls:
  LEFT  hand fist  → ARM ENABLED
  LEFT  hand open  → ARM DISABLED (zero twist sent, arm stops)
  RIGHT hand move  → EE velocity (TwistStamped)
  RIGHT hand pinch → gripper close
  RIGHT hand open  → gripper open

Topics published:
  /servo_node/delta_twist_cmds   (geometry_msgs/TwistStamped)
  /robotiq_gripper_controller/gripper_cmd  (action)

Services called on startup:
  /servo_node/start_servo  (std_srvs/Trigger)
"""

import math
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
from control_msgs.action import GripperCommand
from std_msgs.msg import Bool

from .hand_tracker import HandTracker, HandState


class ServoTeleopNode(Node):

    def __init__(self):
        super().__init__('servo_teleop_node')
        self.cb = ReentrantCallbackGroup()

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('camera_id',           0)
        self.declare_parameter('rate',                30.0)
        self.declare_parameter('show_camera',         True)
        self.declare_parameter('pinch_threshold',     0.07)
        self.declare_parameter('dead_zone',           0.0001) #0.08 reduce noise increase
        # Scale: normalised pixel delta  →  m/s sent to Servo
        # Servo yaml has scale.linear=0.2, so keep this moderate
        self.declare_parameter('linear_scale',        0.1)
        self.declare_parameter('angular_scale',       0.1)
        # Frame that matches robot_link_command_frame in gen3_servo.yaml
        self.declare_parameter('command_frame',       'base_link')

        cam_id              = self.get_parameter('camera_id').value
        self.rate           = self.get_parameter('rate').value
        self.show_camera    = self.get_parameter('show_camera').value
        self.pinch_thr      = self.get_parameter('pinch_threshold').value
        self.dead_zone      = self.get_parameter('dead_zone').value
        self.linear_scale   = self.get_parameter('linear_scale').value
        self.angular_scale  = self.get_parameter('angular_scale').value
        self.command_frame  = self.get_parameter('command_frame').value

        # ── Publishers ───────────────────────────────────────────────
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds', 10)

        # ── Gripper action ───────────────────────────────────────────
        self.gripper_client = ActionClient(
            self, GripperCommand,
            '/robotiq_gripper_controller/gripper_cmd',
            callback_group=self.cb)

        # ── Start MoveIt Servo ───────────────────────────────────────
        self._start_servo()

        # ── Camera ──────────────────────────────────────────────────
        self.cap = cv2.VideoCapture(cam_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera {cam_id}')
            raise RuntimeError(f'Cannot open camera {cam_id}')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # ── Hand tracker ─────────────────────────────────────────────
        self.tracker = HandTracker(pinch_threshold=self.pinch_thr)

        # ── State ────────────────────────────────────────────────────
        self.arm_enabled      = False
        self.left_active      = False
        self.right_active     = False
        self.hand_timeout     = 2.0
        self.left_last_seen   = 0.0
        self.right_last_seen  = 0.0

        self.prev_wrist       = None        # (x, y) normalised
        self.smooth_rot       = 0.0
        self.rot_init         = False

        self.last_gripper_state  = None
        self._gripper_busy       = False

        # ── Timer ────────────────────────────────────────────────────
        self.timer = self.create_timer(1.0 / self.rate, self._timer_cb)

        self.get_logger().info('=' * 50)
        self.get_logger().info('  Kinova Gen3 MoveIt Servo Teleop')
        self.get_logger().info('  LEFT  fist = ARM ON  | open = ARM OFF')
        self.get_logger().info('  RIGHT hand = move EE | pinch = grip')
        self.get_logger().info('  Press Q to quit')
        self.get_logger().info('=' * 50)

    # ---------------------------------------------------------------- #
    #  Start Servo service call                                          #
    # ---------------------------------------------------------------- #

    def _start_servo(self):
        client = self.create_client(Trigger, '/servo_node/start_servo')
        self.get_logger().info('Waiting for /servo_node/start_servo ...')
        if client.wait_for_service(timeout_sec=10.0):
            future = client.call_async(Trigger.Request())
            # Wait briefly — node isn't spinning yet so poll manually
            import time
            deadline = time.time() + 3.0
            while not future.done() and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            if future.done() and future.result().success:
                self.get_logger().info('MoveIt Servo started ✓')
            else:
                self.get_logger().warn('Servo start call failed — may already be running')
        else:
            self.get_logger().warn(
                '/servo_node/start_servo not available — '
                'is servo_kinovagen3.launch.py running?')

    # ---------------------------------------------------------------- #
    #  Main loop                                                         #
    # ---------------------------------------------------------------- #

    def _timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        left, right, annotated = self.tracker.process_frame(frame)
        now = self.get_clock().now().nanoseconds / 1e9
        h, w = annotated.shape[:2]

        # ── LEFT HAND: enable / disable ──────────────────────────────
        if left.detected:
            self.left_last_seen = now
            if not self.left_active:
                self.get_logger().info('Left hand detected')
            self.left_active = True

            was_enabled = self.arm_enabled
            self.arm_enabled = left.fist_closed

            if self.arm_enabled and not was_enabled:
                self.get_logger().info('>>> ARM ENABLED (Servo active)')
                self.prev_wrist = None
                self.rot_init   = False
            elif not self.arm_enabled and was_enabled:
                self.get_logger().info('>>> ARM DISABLED')
                self._publish_zero_twist()   # stop arm immediately
                self.prev_wrist = None

        elif self.left_active and (now - self.left_last_seen > self.hand_timeout):
            self.left_active = False
            self.get_logger().info('Left hand lost')

        # ── RIGHT HAND: velocity control ─────────────────────────────
        if right.detected:
            self.right_last_seen = now
            if not self.right_active:
                self.get_logger().info('Right hand detected')
            self.right_active = True

            if self.arm_enabled:
                twist = self._compute_twist(right)
                self.twist_pub.publish(twist)
            else:
                # Arm disabled — keep sending zero so Servo doesn't drift
                self._publish_zero_twist()

            # ── Gripper (publish only on state change) ──────────────
            gripper_open = not right.gripper_closed
            if gripper_open != self.last_gripper_state:
                self._send_gripper(gripper_open)
                self.last_gripper_state = gripper_open
                self.get_logger().info(
                    f'Gripper: {"OPEN" if gripper_open else "CLOSE"}')

        else:
            # No right hand — send zero twist so arm stops
            if self.arm_enabled:
                self._publish_zero_twist()

            if self.right_active and (now - self.right_last_seen > self.hand_timeout):
                self.right_active = False
                self.prev_wrist   = None
                self.rot_init     = False
                self.get_logger().info('Right hand lost')

        # ── Overlay ──────────────────────────────────────────────────
        self._draw_overlay(annotated, left, right, w, h)
        if self.show_camera:
            cv2.imshow('Kinova Gen3 Servo Teleop', annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

    # ---------------------------------------------------------------- #
    #  Twist computation                                                 #
    # ---------------------------------------------------------------- #

    def _compute_twist(self, right: 'HandState') -> TwistStamped:
        """
        Convert wrist pixel displacement to EE velocity (TwistStamped).

        Mapping (image coords → robot base_link frame):
          image  right (+x)  →  robot  -Y
          image  down  (+y)  →  robot  -Z
          hand rotation      →  robot  +Rz (yaw)
        """
        twist = TwistStamped()
        twist.header.stamp    = self.get_clock().now().to_msg()
        twist.header.frame_id = self.command_frame


        if self.prev_wrist is None:
            self.prev_wrist = (right.wrist_x, right.wrist_y)
            return twist  # zero on first frame



        # Low-pass filter to smooth deltas (reduce noise)
        self.filtered_dx = 0.9 * self.filtered_dx + 0.1 * raw_dx
        self.filtered_dy = 0.9 * self.filtered_dy + 0.1 * raw_dy

        # Use filtered values
        raw_dx = self.filtered_dx
        raw_dy = self.filtered_dy

        # raw_dx = right.wrist_x - self.prev_wrist[0]
        # raw_dy = right.wrist_y - self.prev_wrist[1]
        self.prev_wrist = (right.wrist_x, right.wrist_y)

        # Dead zone
        if abs(raw_dx) < self.dead_zone: raw_dx = 0.0
        if abs(raw_dy) < self.dead_zone: raw_dy = 0.0

        # Clamp to prevent sudden large jumps
        raw_dx = np.clip(raw_dx, -0.05, 0.05)
        raw_dy = np.clip(raw_dy, -0.05, 0.05)

        # Scale to m/s — Servo interprets these as velocities
        # X is fixed depth axis (no depth from RGB camera)
        twist.twist.linear.x  = 0.0
        twist.twist.linear.y  = -raw_dx * self.linear_scale / (1.0 / self.rate)
        twist.twist.linear.z  = -raw_dy * self.linear_scale / (1.0 / self.rate)

        # Rotation from hand roll
        if not self.rot_init:
            self.smooth_rot = right.rotation
            self.rot_init   = True
        else:
            dr = right.rotation - self.smooth_rot
            while dr >  math.pi: dr -= 2 * math.pi
            while dr < -math.pi: dr += 2 * math.pi
            dr = np.clip(dr, -0.02, 0.02)
            self.filtered_rot = 0.9 * self.filtered_rot + 0.1 * dr
            self.smooth_rot += 0.1 * self.filtered_rot

        twist.twist.angular.z = self.smooth_rot * self.angular_scale

        return twist

    def _publish_zero_twist(self):
        twist = TwistStamped()
        twist.header.stamp    = self.get_clock().now().to_msg()
        twist.header.frame_id = self.command_frame
        # All zeros — tells Servo to stop
        self.twist_pub.publish(twist)

    # ---------------------------------------------------------------- #
    #  Gripper                                                           #
    # ---------------------------------------------------------------- #

    def _send_gripper(self, open_gripper: bool):
        if self._gripper_busy:
            return
        if not self.gripper_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn('Gripper action server not ready')
            return
        self._gripper_busy = True
        goal = GripperCommand.Goal()
        goal.command.position   = 0.0 if open_gripper else 0.8
        goal.command.max_effort = 100.0
        f = self.gripper_client.send_goal_async(goal)
        f.add_done_callback(self._gripper_done_cb)

    def _gripper_done_cb(self, future):
        handle = future.result()
        if not handle or not handle.accepted:
            self._gripper_busy = False
            return
        handle.get_result_async().add_done_callback(
            lambda f: setattr(self, '_gripper_busy', False))

    # ---------------------------------------------------------------- #
    #  Overlay                                                           #
    # ---------------------------------------------------------------- #

    def _draw_overlay(self, frame, left, right, w, h):
        if self.arm_enabled:
            cv2.rectangle(frame, (0, 0), (w, 25), (0, 100, 0), -1)
            cv2.putText(frame, 'SERVO ACTIVE — right hand controls EE',
                        (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        else:
            cv2.rectangle(frame, (0, 0), (w, 25), (0, 0, 150), -1)
            cv2.putText(frame, 'SERVO IDLE — close left fist to start',
                        (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        if left.detected:
            txt = 'FIST (ON)' if left.fist_closed else 'OPEN (OFF)'
            cv2.putText(frame, f'LEFT: {txt}', (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            cv2.putText(frame, 'LEFT: ---', (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 2)

        if right.detected:
            col  = (0, 255, 0) if self.arm_enabled else (0, 0, 255)
            grip = 'GRIP' if right.gripper_closed else 'OPEN'
            stat = 'ACTIVE' if self.arm_enabled else 'WAITING'
            cv2.putText(frame, f'RIGHT: {stat} [{grip}]', (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
        else:
            cv2.putText(frame, 'RIGHT: ---', (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 2)

        cv2.putText(frame,
                    'Left fist=ON | Left open=OFF | Pinch=Grip | Q=Quit',
                    (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    # ---------------------------------------------------------------- #
    #  Cleanup                                                           #
    # ---------------------------------------------------------------- #

    def destroy_node(self):
        self._publish_zero_twist()
        self.tracker.destroy()
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoTeleopNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()