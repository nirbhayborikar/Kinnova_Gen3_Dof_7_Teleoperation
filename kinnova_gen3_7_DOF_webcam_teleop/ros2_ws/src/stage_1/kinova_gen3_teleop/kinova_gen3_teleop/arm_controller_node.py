#!/usr/bin/env python3
"""
Arm Controller Node for Kinova Gen3 7-DOF.

Subscribes to target pose and gripper command from hand_pose_publisher.
Calls MoveIt2 /compute_ik for joint angles.
Publishes JointTrajectory to the trajectory controller.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from builtin_interfaces.msg import Duration

import threading
import time


class ArmControllerNode(Node):
    """Subscribes to hand pose and drives Kinova Gen3 via IK."""

    # Kinova Gen3 7-DOF joint names (from URDF)
    ARM_JOINTS = [f'joint_{i}' for i in range(1, 8)]
    # Creates: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']

    # MoveIt2 planning group name (from Kinova MoveIt config SRDF)
    PLANNING_GROUP = 'manipulator'

    # Gripper joint name (Robotiq 2F-85)
    GRIPPER_JOINT = 'finger_joint' # from joint state the top name is consider as  gripper name : ros2 topic echo /joint_states --once

    def __init__(self):
        super().__init__('arm_controller')

        self.cb_group = ReentrantCallbackGroup()

        # ---- Subscribers (from teleop_node) ----
        self.create_subscription(
            PoseStamped, '/target_pose',
            self._pose_cb, 10, callback_group=self.cb_group)
        self.create_subscription(
            Bool, '/gripper_control',
            self._gripper_cb, 10, callback_group=self.cb_group)

        # ---- Joint state subscriber ----
        self.current_joint_state = None
        self.js_lock = threading.Lock()
        self.create_subscription(
            JointState, '/joint_states',
            self._js_cb, 10, callback_group=self.cb_group)

        # ---- Trajectory Publishers ----
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        # Gripper action client
        self.gripper_client = ActionClient(
        self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd',
        callback_group=self.cb_group)



        # ---- IK Service Client ----
        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=self.cb_group)
        self.get_logger().info('Waiting for /compute_ik ...')
        if not self.ik_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error('/compute_ik not available!')
        else:
            self.get_logger().info('/compute_ik ready')

        # ---- Rate limiting ----
        self._arm_busy = False

        self.get_logger().info('Arm Controller Node started (Kinova Gen3 7-DOF)')

    def _js_cb(self, msg):
        with self.js_lock:
            self.current_joint_state = msg

    def _pose_cb(self, msg):
        if self._arm_busy:
            return
        self._arm_busy = True
        threading.Thread(
            target=self._solve_ik,
            args=(msg,),
            daemon=True,
        ).start()

    def _solve_ik(self, pose_msg):
        """Solve IK and publish trajectory (background thread)."""
        try:
            req = GetPositionIK.Request()
            req.ik_request = PositionIKRequest()
            req.ik_request.group_name = self.PLANNING_GROUP
            req.ik_request.avoid_collisions = True
            req.ik_request.pose_stamped = pose_msg

            # Seed with current joint state
            with self.js_lock:
                if self.current_joint_state:
                    rs = RobotState()
                    rs.joint_state = self.current_joint_state
                    req.ik_request.robot_state = rs

            future = self.ik_client.call_async(req)

            # Wait with timeout
            start = time.time()
            while not future.done() and (time.time() - start) < 0.3:
                time.sleep(0.005)

            if not future.done():
                return

            resp = future.result()
            if resp is None or resp.error_code.val != 1:
                self.get_logger().warn(
                    f'IK FAILED - code: {resp.error_code.val if resp else "None"}, '
                    f'target: x={pose_msg.pose.position.x:.3f} '
                    f'y={pose_msg.pose.position.y:.3f} '
                    f'z={pose_msg.pose.position.z:.3f}',
                    throttle_duration_sec=1.0)
                return

            # Extract joint positions
            sol = resp.solution.joint_state
            positions = []
            for jn in self.ARM_JOINTS:
                if jn in sol.name:
                    positions.append(sol.position[sol.name.index(jn)])
                else:
                    return

            # Publish trajectory
            traj = JointTrajectory()
            traj.joint_names = self.ARM_JOINTS
            pt = JointTrajectoryPoint()
            pt.positions = positions
            pt.time_from_start = Duration(sec=0, nanosec=200_000_000)
            traj.points.append(pt)
            self.arm_pub.publish(traj)

            self.get_logger().info(
                f'ARM MOVED - joints: {[f"{p:.2f}" for p in positions]}',
                throttle_duration_sec=1.0)

        except Exception as e:
            self.get_logger().warn(f'IK error: {e}', throttle_duration_sec=2.0)
        finally:
            self._arm_busy = False

    def _gripper_cb(self, msg):
        """msg.data: True=open, False=close"""
        if not self.gripper_client.wait_for_server(timeout_sec=0.1):
            return
        goal = GripperCommand.Goal()
        # Robotiq 2F-85: position 0.0 = open, 0.8 = closed
        goal.command.position = 0.0 if msg.data else 0.8
        goal.command.max_effort = 100.0
        self.gripper_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    executor = MultiThreadedExecutor(num_threads=4)
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
