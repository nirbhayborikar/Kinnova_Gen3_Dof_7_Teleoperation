#!/usr/bin/env python3
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from builtin_interfaces.msg import Duration
from rclpy.duration import Duration as RclpyDuration

class IKWorker(Node):
    """Dedicated node for synchronous IK calls — runs its own executor."""

    def __init__(self):
        super().__init__('ik_worker')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)

        # Executor spins permanently in background — never call
        # spin_until_future_complete() on it from another thread.
        self._thread = threading.Thread(
            target=self._executor.spin, daemon=True)
        self._thread.start()

        self.get_logger().info('Waiting for /compute_ik ...')
        if self.ik_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().info('/compute_ik ready ✓')
        else:
            self.get_logger().error(
                '/compute_ik not available! '
                'Run: ros2 service list | grep compute_ik')

    def solve(self, pose_stamped, joint_state=None, timeout_sec=6.0):
        """
        Blocking IK call. Safe to call from any external thread.

        Uses threading.Event — the already-spinning background executor
        delivers the result without a second spin() call.
        """
        if not self.ik_client.service_is_ready():
            self.get_logger().error(
                '/compute_ik not ready — is MoveIt running? '
                '(ros2 service list | grep compute_ik)')
            return None

        req = GetPositionIK.Request()
        ik  = PositionIKRequest()
        ik.group_name       = 'manipulator'
        ik.ik_link_name     = 'grasping_frame'   # Kinova Gen3 standard EE link
        ik.avoid_collisions = False
        ik.timeout.sec      = 5
        ik.pose_stamped     = pose_stamped
        ik.pose_stamped.header.frame_id = 'base_link'

        if joint_state is not None:
            rs = RobotState()
            rs.joint_state = joint_state
            ik.robot_state = rs

        req.ik_request = ik

        self.get_logger().info('IK request sent ...')
        done = threading.Event()
        future = self.ik_client.call_async(req)
        future.add_done_callback(lambda _: done.set())

        if not done.wait(timeout=timeout_sec):
            self.get_logger().error(
                f'IK timed out after {timeout_sec}s — '
                'MoveIt overloaded or pose unreachable')
            return None

        self.get_logger().info('IK response received ✓')
        return future.result()

    def destroy(self):
        self._executor.shutdown()
        super().destroy_node()


class ArmControllerNode(Node):

    # Kinova Gen3 7-DOF joint names
    ARM_JOINTS = [f'joint_{i}' for i in range(1, 8)]

    # Minimum seconds between IK solves — prevents flooding /compute_ik
    # with every incoming pose (teleop publishes at ~30 Hz).
    IK_MIN_INTERVAL = 0.15   # ~6-7 Hz max IK rate

    def __init__(self, ik_worker: IKWorker):
        super().__init__('arm_controller')
        self.ik  = ik_worker
        self.cb  = ReentrantCallbackGroup()

        self.create_subscription(
            PoseStamped, '/target_pose',
            self._pose_cb, 10, callback_group=self.cb)
        self.create_subscription(
            Bool, '/gripper_control',
            self._gripper_cb, 10, callback_group=self.cb)
        self.create_subscription(
            JointState, '/joint_states',
            self._js_cb, 10, callback_group=self.cb)

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory', 10)

        self.gripper_client = ActionClient(
            self, GripperCommand,
            '/robotiq_gripper_controller/gripper_cmd',
            callback_group=self.cb)

        self.current_js    = None
        self.js_lock       = threading.Lock()
        self._arm_busy     = False
        self._gripper_busy = False
        self._last_ik_time = 0.0   # wall-clock of last IK solve start

        self.get_logger().info('ArmControllerNode ready')

    # ------------------------------------------------------------------ #
    #  Subscriptions                                                       #
    # ------------------------------------------------------------------ #

    def _js_cb(self, msg):
        with self.js_lock:
            self.current_js = msg

    def _pose_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9

        # Skip if arm is busy OR called too soon (rate-limit IK to ~7 Hz)
        if self._arm_busy:
            return
        if (now - self._last_ik_time) < self.IK_MIN_INTERVAL:
            return
        

        self.get_logger().info(
            f'pose_cb -> IK  x={msg.pose.position.x:.3f} '
            f'y={msg.pose.position.y:.3f} '
            f'z={msg.pose.position.z:.3f}')

        self._arm_busy     = True
        self._last_ik_time = now
        threading.Thread(
            target=self._solve_and_move,
            args=(msg,), daemon=True).start()

    
    
    # ------------------------------------------------------------------ #
    #  IK + motion                                                         #
    # ------------------------------------------------------------------ #

    def _solve_and_move(self, pose_msg):
        try:
            with self.js_lock:
                js_snapshot = self.current_js

            self.get_logger().info(
                f'joint_state available: {js_snapshot is not None}')
            
            # before calling ik 
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.707
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 0.707

            resp = self.ik.solve(pose_msg, js_snapshot)

            if resp is None:
                self.get_logger().warn('IK returned None — skipping')
                return

            self.get_logger().info(f'IK error_code: {resp.error_code.val}')

            if resp.error_code.val != 1:
                self.get_logger().warn(
                    f'IK failed (code={resp.error_code.val}) '
                    f'x={pose_msg.pose.position.x:.3f} '
                    f'y={pose_msg.pose.position.y:.3f} '
                    f'z={pose_msg.pose.position.z:.3f}',
                    throttle_duration_sec=1.0)
                return

            sol = resp.solution.joint_state
            positions = []
            for jn in self.ARM_JOINTS:
                if jn in sol.name:
                    positions.append(sol.position[sol.name.index(jn)])
                else:
                    self.get_logger().error(f'IK solution missing joint: {jn}')
                    return

            traj = JointTrajectory()
            traj.header.stamp = self.get_clock().now().to_msg()
            traj.joint_names  = self.ARM_JOINTS
            traj.header.frame_id = 'base_link'
            pt = JointTrajectoryPoint()
            pt.positions      = positions
            pt.velocities = [0.0] * len(positions)
            # 300 ms — short enough for smooth delta tracking
            pt.time_from_start = RclpyDuration(seconds=2).to_msg()
            traj.points.append(pt)
            self.traj_pub.publish(traj) # this is the execution trigger

            self.get_logger().info(
                f'ARM CMD: {[f"{p:.2f}" for p in positions]}',
                throttle_duration_sec=0.5)
            self.get_logger().info(f'IK positions: {[round(p,3) for p in positions]}')
            self.get_logger().info(f'Publishing trajectory...')

        except Exception as e:
            self.get_logger().error(f'_solve_and_move error: {e}')
        finally:
            self._arm_busy = False

    # ------------------------------------------------------------------ #
    #  Gripper                                                             #
    # ------------------------------------------------------------------ #

    def _gripper_cb(self, msg):
        if self._gripper_busy:
            return
        if not self.gripper_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn('Gripper action server not ready')
            return
        self._gripper_busy = True
        goal = GripperCommand.Goal()
        goal.command.position   = 0.0 if msg.data else 0.8   # open=0.0, close=0.8
        goal.command.max_effort = 100.0
        f = self.gripper_client.send_goal_async(goal)
        f.add_done_callback(self._gripper_resp_cb)
        self.get_logger().info(
            f'Gripper goal: {"OPEN" if msg.data else "CLOSE"}')

    def _gripper_resp_cb(self, future):
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().warn('Gripper goal rejected')
            self._gripper_busy = False
            return
        handle.get_result_async().add_done_callback(
            lambda f: setattr(self, '_gripper_busy', False))


# ---------------------------------------------------------------------- #
#  Entry point                                                             #
# ---------------------------------------------------------------------- #

def main(args=None):
    rclpy.init(args=args)

    ik_worker = IKWorker()
    arm_node  = ArmControllerNode(ik_worker)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(arm_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        arm_node.destroy_node()
        ik_worker.destroy()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()