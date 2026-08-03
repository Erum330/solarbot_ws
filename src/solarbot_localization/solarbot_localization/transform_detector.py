import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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

        # Gaussian falloff from center
        x = np.linspace(-1, 1, w)
        y = np.linspace(-1, 1, h)
        xv, yv = np.meshgrid(x, y)
        dist2 = xv**2 + yv**2
        mask = np.exp(-dist2 / 0.25)   # adjust denominator for spread
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
        if theta > -0.0004 and theta < 0.0004:
            theta = 0

        # rotation compensation
        dx_corr = dx - (cx * (1 - np.cos(theta)) + cy * np.sin(theta))
        dy_corr = dy - (cy * (1 - np.cos(theta)) - cx * np.sin(theta))

        return dx_corr, dy_corr, theta, cc


# ============================================================
# ODOMETRY INTEGRATOR
# ============================================================
class OdometryIntegrator:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def update(self, dy, dx, dtheta):
        global_dx = dx * np.cos(self.theta) - dy * np.sin(self.theta)
        global_dy = dx * np.sin(self.theta) + dy * np.cos(self.theta)

        # FIXED AXIS BUG
        self.x += global_dx
        self.y += global_dy

        self.theta += dtheta
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi

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
# IMAGE PREPROCESSOR (FAST)
# ============================================================
class ImagePreprocessor:
    def __init__(self):
        self.crop_w = 470
        self.crop_h = 480
        self.thresh_min = 30
        self.thresh_max = 70

        # PREALLOCATED KERNELS
        self.dilate_kernel = np.ones((3, 3), np.uint8)
        self.glare_kernel = np.ones((5, 5), np.uint8)
        self.conv_kernel = np.ones((5,5), np.uint8)

    def process(self, img_gray):
        h, w = img_gray.shape
        cx, cy = w // 2, h // 2

        # 1. Center crop
        x1 = max(0, cx - self.crop_w // 2)
        y1 = max(0, cy - self.crop_h // 2)
        x2 = min(w, cx + self.crop_w // 2)
        y2 = min(h, cy + self.crop_h // 2)

        img = img_gray[y1:y2, x1:x2]

        # ---------------------------------------------------
        # 2. Contrast normalization (optional but useful)
        # ---------------------------------------------------
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        img_eq = clahe.apply(img)

        # ---------------------------------------------------
        # 3. Gradient magnitude (non-binary edge strength)
        # ---------------------------------------------------
        sobelx = cv2.Sobel(img_eq, cv2.CV_32F, 1, 0, ksize=7)
        sobely = cv2.Sobel(img_eq, cv2.CV_32F, 0, 1, ksize=7)
        grad = cv2.magnitude(sobelx, sobely)

        # Normalize gradient to 0–255
        grad = cv2.normalize(grad, None, 0, 255, cv2.NORM_MINMAX)
        grad = grad.astype(np.uint8)

        # ---------------------------------------------------
        # 4. Directional line enhancement (thick structures)
        # ---------------------------------------------------
        horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 1))
        vertical_kernel   = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 3))

        horiz = cv2.morphologyEx(grad, cv2.MORPH_CLOSE, horizontal_kernel)
        vert  = cv2.morphologyEx(grad, cv2.MORPH_CLOSE, vertical_kernel)

        line_response = cv2.max(horiz, vert)

        # Threshold to binary mask
        _, mask_bin = cv2.threshold(line_response, 30, 255, cv2.THRESH_BINARY)

        # Connected components
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_bin, connectivity=8)
        min_width = 8  # pixels

        mask_filtered = np.zeros_like(mask_bin)
        for i in range(1, num_labels):  # skip background
            x, y, w, h, area = stats[i]
            if w >= min_width or h >= min_width:
                mask_filtered[labels == i] = 255

        # Apply mask to line_response
        line_response = cv2.bitwise_and(line_response, mask_filtered)

        # ---------------------------------------------------
        # 5. Blend with original (KEY STEP — non-binary output)
        # ---------------------------------------------------
        alpha = 0.3   # original weight
        beta  = 1.9   # line emphasis weight

        enhanced = cv2.addWeighted(img, alpha, line_response, beta, 0)

        # Optional: slight sharpening to further emphasize lines
        # enhanced = cv2.GaussianBlur(enhanced, (0,0), 1)
        # enhanced = cv2.addWeighted(enhanced, 1.5, img, -0.5, 0)

        return enhanced


# ============================================================
# MAIN NODE
# ============================================================
class SE2OdometryNode(Node):
    def __init__(self):
        super().__init__('se2_odometry_node')

        self.sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom_cam', 10)
        self.proc_img_pub = self.create_publisher(Image, '/processed_image', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.bridge = CvBridge()
        self.prev = None

        self.estimator = SE2Estimator()
        self.odom = OdometryIntegrator()
        self.calib = PixelToMeter()
        self.preproc = ImagePreprocessor()

        self.skip_count = 0
        self.odom_lost = False

        # THREADING
        self.lock = threading.Lock()
        self.latest_frame = None
        self.latest_stamp = None

        self.worker = threading.Thread(target=self.process_loop, daemon=True)
        self.worker.start()

    # ---------------------------
    # FAST CALLBACK (NON-BLOCKING)
    # ---------------------------
    def callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'mono8')

        with self.lock:
            self.latest_frame = img
            self.latest_stamp = msg.header.stamp

    # ---------------------------
    # WORKER LOOP
    # ---------------------------
    def process_loop(self):
        while rclpy.ok():
            frame = None
            stamp = None

            with self.lock:
                if self.latest_frame is not None:
                    frame = self.latest_frame.copy()
                    stamp = self.latest_stamp
                    self.latest_frame = None

            if frame is None:
                continue

            frame = self.preproc.process(frame)

            # DOWNSCALE FOR SPEED
            img = cv2.resize(frame, None, fx=0.5, fy=0.5)

            if self.prev is None:
                self.prev = img
                continue

            dx, dy, dtheta, score = self.estimator.estimate(self.prev, img)

            # if score < 0.2 or abs(dx) > 40 or abs(dy) > 40:
            if score < 0.5 or self.odom_lost:
                self.get_logger().info(f"Skipping score = {score:.2f}, odom lost : {self.odom_lost}")
                # self.prev = img
                self.skip_count += 1
                if(self.skip_count > 5):
                    self.odom_lost = True
                continue
            else:
                self.skip_count = 0

            # SCALE BACK (because of resize)
            dx *= 2.0
            dy *= 2.0

            self.odom.update(dx, dy, dtheta)
            x_px, y_px, theta = self.odom.get_pose()

            x = self.calib.to_m(x_px)
            y = self.calib.to_m(y_px)

            self.publish_odom(stamp, x, y, theta)

            self.get_logger().info(
                f"x={x:.3f} y={y:.3f} θ={np.degrees(theta):.2f} score={score:.2f}"
            )

            # publish processed image (optional)
            proc_msg = self.bridge.cv2_to_imgmsg(img, encoding='mono8')
            proc_msg.header.stamp = stamp
            self.proc_img_pub.publish(proc_msg)

            self.prev = img

    # ---------------------------
    # ODOM + TF
    # ---------------------------
    def publish_odom(self, stamp, x, y, theta):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y

        qz = np.sin(theta / 2)
        qw = np.cos(theta / 2)

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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()