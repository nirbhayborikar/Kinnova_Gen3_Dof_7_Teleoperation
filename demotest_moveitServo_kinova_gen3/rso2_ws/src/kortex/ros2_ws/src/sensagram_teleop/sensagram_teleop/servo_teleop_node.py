#!/usr/bin/env python3
"""
Kinova Gen3 MoveIt Servo Teleop — MediaPipe hand tracking.

KEY FIX vs previous version:
  Camera capture + MediaPipe inference run in a BACKGROUND THREAD.
  The ROS timer only reads the latest processed result from a shared variable.
  This prevents the ROS executor from blocking on cap.read() / MP inference,
  which was causing the camera freeze.
"""

import math
import sys
import time
import threading

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


from moveit_msgs.srv import ServoCommandType  # Add this import for the service

from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
from control_msgs.action import GripperCommand

# ── Force real pip mediapipe from venv ──────────────────────────────
_REAL_MP = '/opt/venv/lib/python3.12/site-packages'
if _REAL_MP not in sys.path:
    sys.path.insert(0, _REAL_MP)
for _k in list(sys.modules.keys()):
    if _k == 'mediapipe' or _k.startswith('mediapipe.'):
        del sys.modules[_k]
import mediapipe as mp
# ────────────────────────────────────────────────────────────────────


# ════════════════════════════════════════════════════════════════════ #
#  Hand state dataclass                                                #
# ════════════════════════════════════════════════════════════════════ #

class HandState:
    def __init__(self):
        self.detected       = False
        self.wrist_x        = 0.5
        self.wrist_y        = 0.5
        self.fist_closed    = False
        self.gripper_closed = False
        self.rotation       = 0.0


# ════════════════════════════════════════════════════════════════════ #
#  Camera thread — captures + runs MediaPipe, never blocks ROS        #
# ════════════════════════════════════════════════════════════════════ #

class CameraThread(threading.Thread):
    """
    Runs in a daemon thread.
    Reads frames from camera, runs MediaPipe Hands, stores latest result.
    ROS node reads self.left / self.right / self.frame at any time.
    """

    WRIST      = 0
    THUMB_TIP  = 4
    INDEX_TIP  = 8
    INDEX_MCP  = 5
    MIDDLE_TIP = 12
    MIDDLE_MCP = 9
    RING_TIP   = 16
    PINKY_TIP  = 20

    def __init__(self, camera_id: int = 0, pinch_threshold: float = 0.07, show_camera: bool = True):
        super().__init__(daemon=True)
        self.camera_id       = camera_id
        self.pinch_threshold = pinch_threshold
        self._show = show_camera

        self.left  = HandState()
        self.right = HandState()
        self.frame = None           # latest annotated BGR frame

        

        self._lock = threading.Lock()
        self._stop = threading.Event()

        self.mp_hands   = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_styles  = mp.solutions.drawing_styles

    def get_state(self):
        """Thread-safe read of latest hand states + frame."""
        with self._lock:
            return self.left, self.right, self.frame

    def stop(self):
        self._stop.set()

    def run(self):
        cap = cv2.VideoCapture(self.camera_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # Lower buffer size so we always get the freshest frame
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.6,
        )

        while not self._stop.is_set():
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb.flags.writeable = False
            results = hands.process(rgb)
            annotated = frame.copy()

            left  = HandState()
            right = HandState()

            if results.multi_hand_landmarks and results.multi_handedness:
                for lm, handedness in zip(
                        results.multi_hand_landmarks,
                        results.multi_handedness):
                    is_left = (handedness.classification[0].label == 'Right')
                    state = left if is_left else right
                    self._fill_state(state, lm)
                    self.mp_drawing.draw_landmarks(
                        annotated, lm,
                        self.mp_hands.HAND_CONNECTIONS,
                        self.mp_styles.get_default_hand_landmarks_style(),
                        self.mp_styles.get_default_hand_connections_style())

            with self._lock:
                self.left  = left
                self.right = right
                self.frame = annotated
            if self._show:
                cv2.imshow('Kinova Gen3 Servo Teleop', annotated)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self._stop.set()

        cap.release()
        hands.close()

    def _fill_state(self, state: HandState, landmarks):
        lm = landmarks.landmark
        state.detected = True
        state.wrist_x  = lm[self.WRIST].x
        state.wrist_y  = lm[self.WRIST].y

        def dist(a, b):
            return math.hypot(lm[a].x - lm[b].x, lm[a].y - lm[b].y)

        state.fist_closed = (
            dist(self.INDEX_TIP,  self.WRIST) < dist(self.INDEX_MCP,  self.WRIST) and
            dist(self.MIDDLE_TIP, self.WRIST) < dist(self.MIDDLE_MCP, self.WRIST) and
            dist(self.RING_TIP,   self.WRIST) < dist(self.INDEX_MCP,  self.WRIST) and
            dist(self.PINKY_TIP,  self.WRIST) < dist(self.INDEX_MCP,  self.WRIST)
        )
        state.gripper_closed = (
            dist(self.THUMB_TIP, self.INDEX_TIP) < self.pinch_threshold)

        dx = lm[self.MIDDLE_MCP].x - lm[self.WRIST].x
        dy = lm[self.MIDDLE_MCP].y - lm[self.WRIST].y
        state.rotation = math.atan2(-dy, dx)


# ════════════════════════════════════════════════════════════════════ #
#  ROS node — only publishes, never touches the camera                #
# ════════════════════════════════════════════════════════════════════ #

class ServoTeleopNode(Node):

    def __init__(self):
        super().__init__('servo_teleop_node')
        self.cb = ReentrantCallbackGroup()

        # ── Parameters ───────────────────────────────────────────────
        
        self.declare_parameter('camera_id',       0)
        self.declare_parameter('rate',            30.0)
        self.declare_parameter('show_camera',     True)
        self.declare_parameter('pinch_threshold', 0.07)
        self.declare_parameter('dead_zone',       0.008)
        self.declare_parameter('linear_scale',    0.35)
        self.declare_parameter('angular_scale',   0.25)
        self.declare_parameter('command_frame',   'base_link')

        # # ── Set Servo command type to "twist" ─────────────────────────
        # self._set_servo_command_type("twist") # 2 for twist # 0 for unitless


        cam_id           = self.get_parameter('camera_id').value
        self.rate        = self.get_parameter('rate').value
        self.show_camera = self.get_parameter('show_camera').value
        pinch_thr        = self.get_parameter('pinch_threshold').value
        self.dead_zone   = self.get_parameter('dead_zone').value
        self.lin_scale   = self.get_parameter('linear_scale').value
        self.ang_scale   = self.get_parameter('angular_scale').value
        self.cmd_frame   = self.get_parameter('command_frame').value
        self.dt          = 1.0 / self.rate
        

        # ── Publisher ────────────────────────────────────────────────
        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)

        # ── Gripper action ───────────────────────────────────────────
        self.gripper_ac = ActionClient(
            self, GripperCommand,
            '/robotiq_gripper_controller/gripper_cmd',
            callback_group=self.cb)

        # ── Camera thread (non-blocking) ─────────────────────────────
        self.cam_thread = CameraThread(
            camera_id=cam_id,
            pinch_threshold=pinch_thr,
            show_camera=self.show_camera
        )
        self.cam_thread.start()
        self.get_logger().info(f'Camera thread started (id={cam_id}) ✓')

        # ── Start Servo ──────────────────────────────────────────────
        self._start_servo()

         # ── Set Servo command type to "twist" ─────────────────────────
        self._set_servo_command_type(1)


        # ── Runtime state ────────────────────────────────────────────
        self.arm_enabled     = False
        self.left_active     = False
        self.right_active    = False
        self.hand_timeout    = 2.0
        self.left_last_seen  = 0.0
        self.right_last_seen = 0.0
        self.prev_wrist      = None
        self.smooth_rot      = 0.0
        self.rot_init        = False
        self.last_grip_state = None
        self._grip_busy      = False

        # ── ROS timer — just reads latest frame, no camera I/O ───────
        self.timer = self.create_timer(self.dt, self._tick)

        self.get_logger().info('=' * 52)
        self.get_logger().info('  Kinova Gen3 MoveIt Servo Teleop ready')
        self.get_logger().info('  LEFT  fist = ARM ON  |  open = ARM OFF')
        self.get_logger().info('  RIGHT move = EE vel  |  pinch = grip')
        self.get_logger().info('  Press Q in the camera window to quit')
        self.get_logger().info('=' * 52)

    # ---------------------------------------------------------------- #
    #  Set Servo Command Type                                                       #
    # ---------------------------------------------------------------- #

    def _set_servo_command_type(self, command_type: int): # servo take int not string
            client = self.create_client(ServoCommandType, '/servo_node/switch_command_type')
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn('Servo switch_command_type service not available')
                return
            req = ServoCommandType.Request()
            req.command_type = command_type # now int not string
            future = client.call_async(req)
            deadline = time.time() + 3.0
            while not future.done() and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            if future.done() and future.result().success:
                self.get_logger().info(f'Servo command type set to {command_type} ✓')
            else:
                self.get_logger().warn(f'Failed to set servo command type to {command_type}')


            # ── Set Servo command type to "twist" ─────────────────────────
            #self._set_servo_command_type(2)  # Pass 2 for "twist"

    #  Start Servo    

    def _start_servo(self):
        client = self.create_client(Trigger, '/servo_node/start_servo')
        if not client.wait_for_service(timeout_sec=8.0):
            self.get_logger().warn(
                '/servo_node/start_servo not available — '
                'run servo_kinovagen3.launch.py first')
            return
        future = client.call_async(Trigger.Request())
        deadline = time.time() + 3.0
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if future.done() and future.result().success:
            self.get_logger().info('MoveIt Servo started ✓')
        else:
            self.get_logger().warn('Servo start failed — may already be running')

    # ---------------------------------------------------------------- #
    #  ROS tick — reads from camera thread, never blocks               #
    # ---------------------------------------------------------------- #

    def _tick(self):
        left, right, frame = self.cam_thread.get_state()

        if frame is None:
            return  # camera thread not ready yet

        now = self.get_clock().now().nanoseconds / 1e9
        h, w = frame.shape[:2]

        # ── LEFT: enable / disable ───────────────────────────────────
        if left.detected:
            self.left_last_seen = now
            if not self.left_active:
                self.get_logger().info('Left hand detected')
            self.left_active = True

            was = self.arm_enabled
            self.arm_enabled = left.fist_closed

            if self.arm_enabled and not was:
                self.get_logger().info('>>> ARM ENABLED')
                self.prev_wrist = None
                self.rot_init   = False
            elif not self.arm_enabled and was:
                self.get_logger().info('>>> ARM DISABLED')
                self._zero_twist()
                self.prev_wrist = None

        elif self.left_active and (now - self.left_last_seen > self.hand_timeout):
            self.left_active = False
            self.get_logger().info('Left hand lost')

        # ── RIGHT: velocity + gripper ────────────────────────────────
        if right.detected:
            self.right_last_seen = now
            if not self.right_active:
                self.get_logger().info('Right hand detected')
            self.right_active = True

            if self.arm_enabled:
                self.twist_pub.publish(self._compute_twist(right))
            else:
                self._zero_twist()

            grip_open = not right.gripper_closed
            if grip_open != self.last_grip_state:
                self._send_gripper(grip_open)
                self.last_grip_state = grip_open
                self.get_logger().info(
                    f'Gripper → {"OPEN" if grip_open else "CLOSE"}')

        else:
            if self.arm_enabled:
                self._zero_twist()
            if self.right_active and (now - self.right_last_seen > self.hand_timeout):
                self.right_active = False
                self.prev_wrist   = None
                self.rot_init     = False
                self.get_logger().info('Right hand lost')

        # ── Display ──────────────────────────────────────────────────
        self._overlay(frame, left, right, w, h)

    # ---------------------------------------------------------------- #
    #  Twist                                                             #
    # ---------------------------------------------------------------- #

    def _compute_twist(self, right: HandState) -> TwistStamped:
        twist = TwistStamped()
        twist.header.stamp    = self.get_clock().now().to_msg()
        twist.header.frame_id = self.cmd_frame

        if self.prev_wrist is None:
            self.prev_wrist = (right.wrist_x, right.wrist_y)
            return twist

        raw_dx = right.wrist_x - self.prev_wrist[0]
        raw_dy = right.wrist_y - self.prev_wrist[1]
        self.prev_wrist = (right.wrist_x, right.wrist_y)

        if abs(raw_dx) < self.dead_zone: raw_dx = 0.0
        if abs(raw_dy) < self.dead_zone: raw_dy = 0.0

        raw_dx = float(np.clip(raw_dx, -0.06, 0.06))
        raw_dy = float(np.clip(raw_dy, -0.06, 0.06))

        twist.twist.linear.x = 0.0
        twist.twist.linear.y = -raw_dx * self.lin_scale / self.dt
        twist.twist.linear.z = -raw_dy * self.lin_scale / self.dt

        if not self.rot_init:
            self.smooth_rot = right.rotation
            self.rot_init   = True
        else:
            dr = right.rotation - self.smooth_rot
            while dr >  math.pi: dr -= 2 * math.pi
            while dr < -math.pi: dr += 2 * math.pi
            self.smooth_rot += 0.3 * float(np.clip(dr, -0.1, 0.1))

        twist.twist.angular.z = self.smooth_rot * self.ang_scale
        return twist

    def _zero_twist(self):
        t = TwistStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = self.cmd_frame
        self.twist_pub.publish(t)

    # ---------------------------------------------------------------- #
    #  Gripper                                                           #
    # ---------------------------------------------------------------- #

    def _send_gripper(self, open_gripper: bool):
        if self._grip_busy:
            return
        if not self.gripper_ac.wait_for_server(timeout_sec=0.1):
            return
        self._grip_busy = True
        goal = GripperCommand.Goal()
        goal.command.position   = 0.0 if open_gripper else 0.8
        goal.command.max_effort = 100.0
        f = self.gripper_ac.send_goal_async(goal)
        f.add_done_callback(self._grip_done_cb)

    def _grip_done_cb(self, future):
        handle = future.result()
        if not handle or not handle.accepted:
            self._grip_busy = False
            return
        handle.get_result_async().add_done_callback(
            lambda _: setattr(self, '_grip_busy', False))

    # ---------------------------------------------------------------- #
    #  Overlay                                                           #
    # ---------------------------------------------------------------- #

    def _overlay(self, frame, left: HandState, right: HandState, w, h):
        if self.arm_enabled:
            cv2.rectangle(frame, (0, 0), (w, 28), (0, 120, 0), -1)
            cv2.putText(frame, 'SERVO ACTIVE — right hand controls EE',
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        else:
            cv2.rectangle(frame, (0, 0), (w, 28), (0, 0, 160), -1)
            cv2.putText(frame, 'SERVO IDLE — close left fist to start',
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        lc = (0, 255, 255) if left.detected else (100, 100, 100)
        lt = ('FIST(ON)' if left.fist_closed else 'OPEN(OFF)') if left.detected else '---'
        cv2.putText(frame, f'LEFT : {lt}', (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, lc, 2)

        if right.detected:
            rc   = (0, 255, 0) if self.arm_enabled else (60, 60, 255)
            grip = 'PINCH' if right.gripper_closed else 'OPEN'
            stat = 'ACTIVE' if self.arm_enabled else 'WAITING'
            cv2.putText(frame, f'RIGHT: {stat} | grip={grip}', (10, 85),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, rc, 2)
        else:
            cv2.putText(frame, 'RIGHT: ---', (10, 85),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 2)

        cv2.putText(frame,
                    'L-fist=ON | L-open=OFF | R-pinch=Grip | Q=Quit',
                    (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    # ---------------------------------------------------------------- #
    #  Cleanup                                                           #
    # ---------------------------------------------------------------- #

    def destroy_node(self):
        self._zero_twist()
        self.cam_thread.stop()
        cv2.destroyAllWindows()
        super().destroy_node()


# ════════════════════════════════════════════════════════════════════ #
#  Entry point                                                          #
# ════════════════════════════════════════════════════════════════════ #

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