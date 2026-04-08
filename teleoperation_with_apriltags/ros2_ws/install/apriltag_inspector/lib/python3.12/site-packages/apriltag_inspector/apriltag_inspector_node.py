"""
apriltag_inspector_node.py  (v2 — tag center + projected axis cross)
=====================================================================
New in v2:
  - Subscribes to camera_info → real fx, fy, cx, cy (no more guessing)
  - Finds tag CENTER in pixel coordinates by projecting (tx,ty,tz)
  - Draws 2D axis cross (X red, Y green) projected INTO the tag plane
  - Draws a crosshair + circle at the exact pixel center of the tag
  - Shows center pixel (u,v) and 3D center (x,y,z) in the HUD
  - Publishes center as PointStamped on /apriltag_inspector/center
    so the follower node can subscribe and use it directly

Why the CENTER matters for the follower:
  The TF frame origin IS the tag center. So the 3D center in base_link
  coords is exactly what you feed to the follower as the target position.

Axis cross on the tag:
  Tag X = right along tag face  → drawn RED with arrow
  Tag Y = up along tag face     → drawn GREEN with arrow
  Tag Z = out of tag toward you → NOT drawn (that is the depth axis)
  When the tag rotates, the arrows rotate with it in the image.
  This gives instant visual feedback on orientation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, TransformStamped
from cv_bridge import CvBridge
import tf2_ros
import numpy as np
import cv2
import math


# ── helpers ───────────────────────────────────────────────────────────────────

def quat_to_euler_deg(qx, qy, qz, qw):
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.degrees(math.atan2(sinr, cosr))
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.degrees(math.asin(sinp))
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.degrees(math.atan2(siny, cosy))
    return roll, pitch, yaw


def quat_to_rot(qx, qy, qz, qw):
    """Quaternion → 3×3 rotation matrix (tag_frame axes in camera_frame)."""
    return np.array([
        [1-2*(qy**2+qz**2),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [  2*(qx*qy+qz*qw), 1-2*(qx**2+qz**2),   2*(qy*qz-qx*qw)],
        [  2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx**2+qy**2)],
    ], dtype=float)



def project_pt(pt3d, fx, fy, cx, cy):
    """
    This is the core of Pin-hole Camera Geometry. It takes a 3D point (X,Y,Z) and transforms it into 2D pixel 
    coordinates (u,v) using the focal lengths (fx​,fy​) and principal points (cx​,cy​):
    Project 3D camera-frame point → (u, v) pixels. Returns None if behind camera."""
    if pt3d[2] < 0.01:
        return None
    return (int(fx * pt3d[0] / pt3d[2] + cx),
            int(fy * pt3d[1] / pt3d[2] + cy))


def parse_tf(t: TransformStamped):
    tr = t.transform.translation
    ro = t.transform.rotation
    roll, pitch, yaw = quat_to_euler_deg(ro.x, ro.y, ro.z, ro.w)
    dist = math.sqrt(tr.x**2 + tr.y**2 + tr.z**2)
    return dict(x=tr.x, y=tr.y, z=tr.z,
                roll=roll, pitch=pitch, yaw=yaw, dist=dist,
                qx=ro.x, qy=ro.y, qz=ro.z, qw=ro.w)


# ── node ──────────────────────────────────────────────────────────────────────

class AprilTagInspector(Node):

    def __init__(self):
        super().__init__('apriltag_inspector')

        self.declare_parameter('tag_frame',    'tag36h11:0')
        self.declare_parameter('camera_frame', 'camera_color_frame')
        self.declare_parameter('base_frame',   'base_link')
        self.declare_parameter('image_topic',  '/kortex_vision/color/image_raw')
        self.declare_parameter('info_topic',   '/kortex_vision/color/camera_info')
        self.declare_parameter('show_window',  True)
        self.declare_parameter('axis_length',  0.05)   # metres drawn per axis

        # Camera intrinsics — updated from /camera_info
        self.fx = 600.0; self.fy = 600.0
        self.cx = 320.0; self.cy = 240.0
        self.got_K = False

        # TF2
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()

        # Subscriptions
        self.create_subscription(
            CameraInfo,
            self.get_parameter('info_topic').value,
            self._info_cb, 1)
        self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self._image_cb, 10)

        # Publications
        self.img_pub    = self.create_publisher(Image,        '/apriltag_inspector/annotated_image', 10)
        self.center_pub = self.create_publisher(PointStamped, '/apriltag_inspector/center',          10)

        # State
        self.cam_data    = None
        self.base_data   = None
        self.tag_visible = False
        self.last_seen   = self.get_clock().now()

        self.create_timer(0.05, self._tf_poll)   # 20 Hz TF poll

        self.get_logger().info('AprilTag Inspector v2 ready')

    # ── camera intrinsics ─────────────────────────────────────────────────────
    def _info_cb(self, msg: CameraInfo):
        if not self.got_K:
            K = msg.k
            self.fx = K[0]; self.fy = K[4]
            self.cx = K[2]; self.cy = K[5]
            self.got_K = True
            self.get_logger().info(
                f'Intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} '
                f'cx={self.cx:.1f} cy={self.cy:.1f}')

    # ── TF poll ───────────────────────────────────────────────────────────────
    def _tf_poll(self):
        tag  = self.get_parameter('tag_frame').value
        cam  = self.get_parameter('camera_frame').value
        base = self.get_parameter('base_frame').value
        to   = rclpy.duration.Duration(seconds=0.04)
        ts   = rclpy.time.Time()

        try:
            self.cam_data    = parse_tf(self.tf_buffer.lookup_transform(cam, tag, ts, to))
            self.tag_visible = True
            self.last_seen   = self.get_clock().now()
        except Exception:
            if (self.get_clock().now()-self.last_seen).nanoseconds*1e-9 > 0.5:
                self.tag_visible = False
                self.cam_data = self.base_data = None
            return

        try:
            t_base = self.tf_buffer.lookup_transform(base, tag, ts, to)
            self.base_data = parse_tf(t_base)
            # Publish center in base_link for follower node
            p = PointStamped()
            p.header.stamp    = self.get_clock().now().to_msg()
            p.header.frame_id = base
            p.point.x = self.base_data['x']
            p.point.y = self.base_data['y']
            p.point.z = self.base_data['z']
            self.center_pub.publish(p)
        except Exception:
            self.base_data = None

    # ── image callback ────────────────────────────────────────────────────────
    def _image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        h, w = frame.shape[:2]
        d = self.cam_data

        if self.tag_visible and d:
            # ── Project tag center to pixel coords ────────────────────────
            center_px = project_pt(
                [d['x'], d['y'], d['z']],
                self.fx, self.fy, self.cx, self.cy)

            if center_px:
                # ── Axis cross projected onto tag face ────────────────────
                self._draw_axis_cross(frame, d, center_px)
                # ── Crosshair at center ───────────────────────────────────
                self._draw_crosshair(frame, center_px)

            # ── Info panel bottom-right ───────────────────────────────────
            self._draw_hud(frame, d, self.base_data, center_px, w, h)
        else:
            self._draw_no_tag(frame, w, h)

        # Title bar
        tag    = self.get_parameter('tag_frame').value
        status = 'TRACKING' if self.tag_visible else 'SEARCHING'
        color  = (60, 220, 60) if self.tag_visible else (60, 60, 200)
        intr   = 'real K' if self.got_K else 'approx K'
        cv2.putText(frame, f'{tag}  |  {status}  |  {intr}',
                    (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.44, color, 1,
                    cv2.LINE_AA)

        try:
            out = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            out.header = msg.header
            self.img_pub.publish(out)
        except Exception:
            pass

        if self.get_parameter('show_window').value:
            cv2.imshow('AprilTag Inspector', frame)
            cv2.waitKey(1)

    # ── axis cross ────────────────────────────────────────────────────────────
    def _draw_axis_cross(self, frame, d, center_px):
        """
        Draw the tag's own X and Y axes projected into the image.
        Uses the rotation matrix so the arrows physically point along
        the tag face — they rotate as the tag rotates.

        Math:
          R  = rotation matrix built from the tag's quaternion
          R[:,0] = tag's X axis direction in camera frame
          R[:,1] = tag's Y axis direction in camera frame
          tip in camera frame = tag_origin + R[:,i] * axis_length
          tip in pixels       = project(tip_cam)
        """
        aln = self.get_parameter('axis_length').value
        R   = quat_to_rot(d['qx'], d['qy'], d['qz'], d['qw'])
        t   = np.array([d['x'], d['y'], d['z']])

        # Tips of each axis in camera frame
        tip_x_cam = t + R[:, 0] * aln
        tip_y_cam = t + R[:, 1] * aln

        tip_x_px = project_pt(tip_x_cam, self.fx, self.fy, self.cx, self.cy)
        tip_y_px = project_pt(tip_y_cam, self.fx, self.fy, self.cx, self.cy)

        u0, v0 = center_px

        if tip_x_px:
            cv2.arrowedLine(frame, (u0, v0), tip_x_px,
                            (40, 40, 230), 2, cv2.LINE_AA, tipLength=0.25)
            cv2.putText(frame, 'X',
                        (tip_x_px[0]+6, tip_x_px[1]-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                        (40, 40, 230), 1, cv2.LINE_AA)

        if tip_y_px:
            cv2.arrowedLine(frame, (u0, v0), tip_y_px,
                            (40, 210, 40), 2, cv2.LINE_AA, tipLength=0.25)
            cv2.putText(frame, 'Y',
                        (tip_y_px[0]+6, tip_y_px[1]-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                        (40, 210, 40), 1, cv2.LINE_AA)

    # ── crosshair ─────────────────────────────────────────────────────────────
    def _draw_crosshair(self, frame, center_px):
        """
        Draws concentric rings + 4-arm crosshair at the tag center pixel.
        The cyan dot is the EXACT point the follower node aims for.
        """
        u, v = center_px
        r1, r2, arm = 10, 22, 34

        # Outer thin white ring
        cv2.circle(frame, (u, v), r2, (255, 255, 255), 1, cv2.LINE_AA)
        # Inner cyan ring
        cv2.circle(frame, (u, v), r1, (0, 220, 255), 1, cv2.LINE_AA)
        # Filled cyan dot at exact center
        cv2.circle(frame, (u, v),  4, (0, 220, 255), -1, cv2.LINE_AA)

        gap = r1 + 4   # gap between ring edge and arm start
        cv2.line(frame, (u,     v-gap), (u,     v-arm),  (0, 220, 255), 1, cv2.LINE_AA)
        cv2.line(frame, (u,     v+gap), (u,     v+arm),  (0, 220, 255), 1, cv2.LINE_AA)
        cv2.line(frame, (u-gap, v),     (u-arm, v),       (0, 220, 255), 1, cv2.LINE_AA)
        cv2.line(frame, (u+gap, v),     (u+arm, v),       (0, 220, 255), 1, cv2.LINE_AA)

        # Pixel label
        cv2.putText(frame, f'({u},{v})px',
                    (u+28, v+16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                    (0, 220, 255), 1, cv2.LINE_AA)

    # ── HUD panel ─────────────────────────────────────────────────────────────
    def _draw_hud(self, frame, d, db, center_px, w, h):
        L = []   # (text, color, is_header)

        # Center block
        L.append(('CENTER',                      None,          True))
        if center_px:
            L.append((f'pixel ({center_px[0]},{center_px[1]})', (0,220,255), False))
        L.append((f'dist  {d["dist"]*100:.1f} cm', (0,220,255), False))
        L.append(('', None, False))

        # Camera frame
        L.append(('CAMERA FRAME',   (180,180,180), True))
        L.append((f'X {d["x"]*100:+6.1f} cm',    (80,100,255),  False))
        L.append((f'Y {d["y"]*100:+6.1f} cm',    (80,200,80),   False))
        L.append((f'Z {d["z"]*100:+6.1f} cm',    (255,100,100), False))
        L.append((f'Roll  {d["roll"]:+5.1f}°',   (200,200,80),  False))
        L.append((f'Pitch {d["pitch"]:+5.1f}°',  (200,200,80),  False))
        L.append((f'Yaw   {d["yaw"]:+5.1f}°',    (200,200,80),  False))
        L.append(('', None, False))

        # Base link frame
        if db:
            L.append(('BASE_LINK FRAME', (180,180,180), True))
            L.append((f'X {db["x"]*100:+6.1f} cm',   (80,100,255),  False))
            L.append((f'Y {db["y"]*100:+6.1f} cm',   (80,200,80),   False))
            L.append((f'Z {db["z"]*100:+6.1f} cm',   (255,100,100), False))
            L.append((f'Roll  {db["roll"]:+5.1f}°',  (200,200,80),  False))
            L.append((f'Pitch {db["pitch"]:+5.1f}°', (200,200,80),  False))
            L.append((f'Yaw   {db["yaw"]:+5.1f}°',   (200,200,80),  False))
            L.append(('', None, False))

        L.append(('AXIS CROSS', (120,120,120), True))
        L.append(('red=X  green=Y', (100,100,100), False))

        # Panel geometry
        lh = 18; pad = 8; panel_w = 188
        panel_h = sum(lh if txt else lh//2 for txt,_,_ in L) + pad*2
        px = w - panel_w - 10
        py = h - panel_h - 10

        ov = frame.copy()
        cv2.rectangle(ov, (px,py), (px+panel_w, py+panel_h), (12,12,12), -1)
        cv2.addWeighted(ov, 0.80, frame, 0.20, 0, frame)
        cv2.rectangle(frame, (px,py), (px+panel_w, py+panel_h), (70,70,70), 1)

        y = py + pad + lh
        for text, color, header in L:
            if not text:
                y += lh//2; continue
            if color is None:
                color = (200,200,200)
            fs = 0.36 if header else 0.40
            fw = 2 if header else 1
            cv2.putText(frame, text, (px+pad, y),
                        cv2.FONT_HERSHEY_SIMPLEX, fs, color, fw,
                        cv2.LINE_AA)
            y += lh

    # ── no-tag banner ─────────────────────────────────────────────────────────
    def _draw_no_tag(self, frame, w, h):
        ov = frame.copy()
        cv2.rectangle(ov, (0,0), (w,44), (0,0,45), -1)
        cv2.addWeighted(ov, 0.7, frame, 0.3, 0, frame)
        cv2.putText(frame, 'TAG NOT DETECTED — move tag into view',
                    (16, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.53,
                    (60,60,255), 1, cv2.LINE_AA)

    def destroy_node(self):
        if self.get_parameter('show_window').value:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagInspector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()








# ros2 run apriltag_inspector apriltag_inspector_node \
#   --ros-args \
#   -p tag_frame:=tag36h11:0 \
#   -p camera_frame:=camera_color_frame \
#   -p base_frame:=base_link \
#   -p image_topic:=/kortex_vision/color/image_raw \
#   -p show_window:=true
    
    # To view the annotated image in RViz instead of a window:
# Set show_window:=false, then subscribe to:
# /apriltag_inspector/annotated_image