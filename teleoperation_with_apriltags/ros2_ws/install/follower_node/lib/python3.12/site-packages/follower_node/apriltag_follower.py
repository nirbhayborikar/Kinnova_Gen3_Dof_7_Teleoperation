"""
follower_node.py — Kinova Gen3 AprilTag Visual Servoing
Standalone node, separate from the kortex stack.

Reads from kortex stack (via shared ROS_DOMAIN_ID + host network):
  /tf                        tag36h11:0 pose (from apriltag container)
  /tf                        tool_frame pose  (from kortex container)

Writes to kortex stack:
  /twist_controller/commands geometry_msgs/Twist

Zero kortex dependencies in this package — pure rclpy + tf2.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros
import numpy as np


class AprilTagFollower(Node):

    def __init__(self):
        super().__init__('apriltag_follower')

        # All params loaded from config/follower_params.yaml
        self.declare_parameter('kp',               0.8)
        self.declare_parameter('alpha',            0.15)
        self.declare_parameter('max_vel',          0.08)
        self.declare_parameter('deadband',         0.012)
        self.declare_parameter('safe_offset',      0.15)
        self.declare_parameter('x_min',            0.20)
        self.declare_parameter('x_max',            0.70)
        self.declare_parameter('y_min',           -0.40)
        self.declare_parameter('y_max',            0.40)
        self.declare_parameter('z_min',            0.10)
        self.declare_parameter('z_max',            0.60)
        self.declare_parameter('tag_loss_timeout', 0.50)
        self.declare_parameter('control_rate',    20.0)
        self.declare_parameter('tag_frame',       'tag36h11:0')
        self.declare_parameter('base_frame',      'base_link')
        self.declare_parameter('eef_frame',       'tool_frame')

        # TF2 — reads /tf and /tf_static from the network
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher — writes to kortex twist_controller on the shared network
        self.twist_pub = self.create_publisher(
            Twist, '/twist_controller/commands', 10) # this controller only active in real hardware

        # EMA state
        self.fx = 0.45
        self.fy = 0.00
        self.fz = 0.35

        self.last_seen       = self.get_clock().now()
        self._moving         = False
        self._warmup         = 0

        rate = self.get_parameter('control_rate').value
        self.timer = self.create_timer(1.0 / rate, self._loop)

        self.get_logger().info(
            f'\n'
            f'  tag_frame  : {self.get_parameter("tag_frame").value}\n'
            f'  base_frame : {self.get_parameter("base_frame").value}\n'
            f'  eef_frame  : {self.get_parameter("eef_frame").value}\n'
            f'  kp={self.get_parameter("kp").value}  '
            f'alpha={self.get_parameter("alpha").value}  '
            f'max_vel={self.get_parameter("max_vel").value} m/s'
        )

    def _p(self, name):
        return self.get_parameter(name).value

    def _loop(self):
        now = self.get_clock().now()

        # 1 — Tag pose in base_link
        try:
            t = self.tf_buffer.lookup_transform(
                self._p('base_frame'), self._p('tag_frame'),
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except Exception:
            elapsed = (now - self.last_seen).nanoseconds * 1e-9
            if elapsed > self._p('tag_loss_timeout') and self._moving:
                self.get_logger().warn(
                    f'Tag lost {elapsed:.1f}s — stopping.')
                self.twist_pub.publish(Twist())
                self._moving  = False
                self._warmup  = 0
            return

        self.last_seen = now
        tx = t.transform.translation.x
        ty = t.transform.translation.y
        tz = t.transform.translation.z

        # 2 — EMA filter (kills jitter)
        a = self._p('alpha')
        self.fx = a * tx + (1 - a) * self.fx
        self.fy = a * ty + (1 - a) * self.fy
        self.fz = a * tz + (1 - a) * self.fz

        # 3 — Safety offset (stay back from tag)
        tgt_x = self.fx
        tgt_y = self.fy
        tgt_z = self.fz + self._p('safe_offset')

        # 4 — Workspace clamp (hard safety)
        tgt_x = float(np.clip(tgt_x, self._p('x_min'), self._p('x_max')))
        tgt_y = float(np.clip(tgt_y, self._p('y_min'), self._p('y_max')))
        tgt_z = float(np.clip(tgt_z, self._p('z_min'), self._p('z_max')))

        # 5 — Current EEF pose
        try:
            e = self.tf_buffer.lookup_transform(
                self._p('base_frame'), self._p('eef_frame'),
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except Exception:
            return

        ex = tgt_x - e.transform.translation.x
        ey = tgt_y - e.transform.translation.y
        ez = tgt_z - e.transform.translation.z
        err = float(np.linalg.norm([ex, ey, ez]))

        # 6 — P-controller → Twist
        twist = Twist()
        if err > self._p('deadband'):
            self._warmup += 1
            if self._warmup < 3:     # 3-frame warmup prevents lurch
                return
            mv = self._p('max_vel')
            kp = self._p('kp')
            twist.linear.x = float(np.clip(kp * ex, -mv, mv))
            twist.linear.y = float(np.clip(kp * ey, -mv, mv))
            twist.linear.z = float(np.clip(kp * ez, -mv, mv))
            self._moving = True
        else:
            self._moving  = False
            self._warmup  = 0

        self.twist_pub.publish(twist)

        # 7 — Log every ~2 s
        if (now.nanoseconds // int(2e9)) % 2 == 0:
            self.get_logger().info(
                f'raw=[{tx:.3f},{ty:.3f},{tz:.3f}] '
                f'filt=[{self.fx:.3f},{self.fy:.3f},{self.fz:.3f}] '
                f'err={err:.4f}m '
                f'v=[{twist.linear.x:.3f},'
                f'{twist.linear.y:.3f},{twist.linear.z:.3f}]'
            )

    def destroy_node(self):
        self.get_logger().info('Shutdown — zeroing twist.')
        self.twist_pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()