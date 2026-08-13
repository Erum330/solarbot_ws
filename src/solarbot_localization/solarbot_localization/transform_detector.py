#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf2_ros
import threading


# ============================================================
# SE(2) ESTIMATOR (ECC-based)
# ============================================================
class SE2Estimator:
    def __init__(self):
        self.warp_mode = cv2.MOTION_EUCLIDEAN
        self.criteria = (
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
            30,
            1e-5
        )

    def estimate(self, img1, img2):
        h, w = img1.shape
        cx, cy = w / 2.0, h / 2.0

        warp = np.eye(2, 3, dtype=np.float32)

        # Gaussian falloff mask centered on image
        x = np.linspace(-1, 1, w)
        y = np.linspace(-1, 1, h)
        xv, yv = np.meshgrid(x, y)
        dist2 = xv**2 + yv**2
        mask = np.exp(-dist2 / 0.25)
        mask = (mask * 255).astype(np.uint8)

        try:
            cc, warp = cv2.findTransformECC(
                img1,
                img2,
                warp,
                self.warp_mode,
                self.criteria,
                mask,
                5
            )
        except cv2.error:
            return 0.0, 0.0, 0.0, 0.0

        dx = warp[0, 2]
        dy = warp[1, 2]
        theta = np.arctan2(warp[1, 0], warp[0, 0])
        if -0.0004 < theta < 0.0004:
            theta = 0.0

        # Rotation compensation for image translation
        dx_corr = dx - (cx * (1 - np.cos(theta)) + cy * np.sin(theta))
        dy_corr = dy - (cy * (1 - np.cos(theta)) - cx * np.sin(theta))

        return dx_corr, dy_corr, theta, cc


# ============================================================
# ODOMETRY INTEGRATOR (Fused with IMU Heading)
# ============================================================
class OdometryIntegrator:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def update(self, dx, dy, imu_theta=None, dtheta_vis=0.0):
        # Prefer direct IMU yaw if available; fallback to visual dtheta
        if imu_theta is not None:
            self.theta = imu_theta
        else:
            self.theta += dtheta_vis

        # Normalize heading to [-pi, pi]
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi

        # Transform body displacements (dx=Forward, dy=Left/Right) into global odom frame
        global_dx = dx * np.cos(self.theta) - dy * np.sin(self.theta)
        global_dy = dx * np.sin(self.theta) + dy * np.cos(self.theta)

        self.x += global_dx
        self.y += global_dy

    def get_pose(self):
        return self.x, self.y, self.theta


# ============================================================
# CALIBRATION
# ============================================================
class PixelToMeter:
    def __init__(self, px_per_meter=2875):
        self.scale = px_per_meter

    def to_m(self, px):
        return px / self.scale


# ============================================================
# IMAGE PREPROCESSOR
# ============================================================
class ImagePreprocessor:
    def __init__(self):
        self.crop_w = 470
        self.crop_h = 480

    def process(self, img_gray):
        h, w = img_gray.shape
        cx, cy = w // 2, h // 2

        # 1. Center crop
        x1 = max(0, cx - self.crop_w // 2)
        y1 = max(0, cy - self.crop_h // 2)
        x2 = min(w, cx + self.crop_w // 2)
        y2 = min(h, cy + self.crop_h // 2)

        img = img_gray[y1:y2, x1:x2]

        # 2. Contrast normalization (CLAHE)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        img_eq = clahe.apply(img)

        # 3. Gradient magnitude
        sobelx = cv2.Sobel(img_eq, cv2.CV_32F, 1, 0, ksize=7)
        sobely = cv2.Sobel(img_eq, cv2.CV_32F, 0, 1, ksize=7)
        grad = cv2.magnitude(sobelx, sobely)

        grad = cv2.normalize(grad, None, 0, 255, cv2.NORM_MINMAX)
        grad = grad.astype(np.uint8)

        # 4. Directional line enhancement
        horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 1))
        vertical_kernel   = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 3))

        horiz = cv2.morphologyEx(grad, cv2.MORPH_CLOSE, horizontal_kernel)
        vert  = cv2.morphologyEx(grad, cv2.MORPH_CLOSE, vertical_kernel)

        line_response = cv2.max(horiz, vert)

        _, mask_bin = cv2.threshold(line_response, 30, 255, cv2.THRESH_BINARY)

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask_bin, connectivity=8)
        min_width = 8

        mask_filtered = np.zeros_like(mask_bin)
        for i in range(1, num_labels):
            _, _, w_comp, h_comp, _ = stats[i]
            if w_comp >= min_width or h_comp >= min_width:
                mask_filtered[labels == i] = 255

        line_response = cv2.bitwise_and(line_response, mask_filtered)

        # 5. Blend
        alpha = 0.3
        beta  = 1.9
        enhanced = cv2.addWeighted(img, alpha, line_response, beta, 0)

        return enhanced


# ============================================================
# MAIN NODE
# ============================================================
class SE2OdometryNode(Node):
    def __init__(self):
        super().__init__('se2_odometry_node')

        # Camera Subscription
        self.sub_cam = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # IMU Subscription
        self.sub_imu = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom_cam', 10)
        self.proc_img_pub = self.create_publisher(Image, '/processed_image', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.bridge = CvBridge()
        self.prev = None

        self.estimator = SE2Estimator()
        self.odom = OdometryIntegrator()
        self.calib = PixelToMeter(px_per_meter=2875)
        self.preproc = ImagePreprocessor()

        self.skip_count = 0
        self.odom_lost = False

        # State Variables & Thread Locking
        self.lock = threading.Lock()
        self.latest_frame = None
        self.latest_stamp = None
        
        self.current_imu_yaw = None
        self.have_imu = False

        self.worker = threading.Thread(target=self.process_loop, daemon=True)
        self.worker.start()

        self.get_logger().info('🚀 SE(2) Visual-Inertial Odometry Node Active (Axes Mapped to ROS ENU)')

    # ---------------------------
    # FAST CALLBACKS
    # ---------------------------
    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'mono8')

        with self.lock:
            self.latest_frame = img
            self.latest_stamp = msg.header.stamp

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        with self.lock:
            self.current_imu_yaw = yaw
            self.have_imu = True

    # ---------------------------
    # WORKER LOOP
    # ---------------------------
    def process_loop(self):
        while rclpy.ok():
            frame = None
            stamp = None
            imu_yaw = None

            with self.lock:
                if self.latest_frame is not None:
                    frame = self.latest_frame.copy()
                    stamp = self.latest_stamp
                    imu_yaw = self.current_imu_yaw
                    self.latest_frame = None

            if frame is None:
                continue

            frame = self.preproc.process(frame)

            # Downscale image for fast ECC estimation
            img = cv2.resize(frame, None, fx=0.5, fy=0.5)

            if self.prev is None:
                self.prev = img
                continue

            img_dx, img_dy, dtheta_vis, score = self.estimator.estimate(self.prev, img)

            if score < 0.5 or self.odom_lost:
                self.get_logger().info(f"Skipping frame score = {score:.2f}, odom lost : {self.odom_lost}")
                self.skip_count += 1
                if self.skip_count > 5:
                    self.odom_lost = True
                continue
            else:
                self.skip_count = 0

            # Scale translation back due to image downscaling (fx=0.5, fy=0.5)
            img_dx *= 2.0
            img_dy *= 2.0

            # CAMERA-TO-CHASSIS AXIS MAP:
            # - Forward robot motion moves camera pixels DOWN (+img_dy) -> Chassis +X
            # - Leftward robot motion moves camera pixels RIGHT (+img_dx) -> Chassis +Y
            body_dx = img_dy
            body_dy = img_dx

            # Update odometry integrator using mapped axes and fused IMU yaw
            self.odom.update(body_dx, body_dy, imu_theta=imu_yaw, dtheta_vis=dtheta_vis)
            x_px, y_px, theta = self.odom.get_pose()

            x = self.calib.to_m(x_px)
            y = self.calib.to_m(y_px)

            self.publish_odom(stamp, x, y, theta)

            self.get_logger().info(
                f"x={x:.3f} y={y:.3f} θ={np.degrees(theta):.2f}° score={score:.2f} [IMU Used: {imu_yaw is not None}]"
            )

            # Publish processed diagnostic image
            proc_msg = self.bridge.cv2_to_imgmsg(img, encoding='mono8')
            proc_msg.header.stamp = stamp
            self.proc_img_pub.publish(proc_msg)

            self.prev = img

    # ---------------------------
    # ODOM + TF PUBLISHER
    # ---------------------------
    def publish_odom(self, stamp, x, y, theta):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y

        qz = np.sin(theta / 2.0)
        qw = np.cos(theta / 2.0)

        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


# ============================================================
# MAIN
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = SE2OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()