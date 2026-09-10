import numpy as np
import cv2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError, CvBridge

CAMERA_INFO_TOPIC = "/camera/color/camera_info"
CAMERA_COLOR_TOPIC = "/camera/color/image_raw"

def image_process(image, ds_factor, row_crop_top, row_crop_bottom, col_crop_left, col_crop_right):
    if image is None:
        raise Exception("Camera is not sending images!")
    h, w = image.shape[:2] # Run camera node: ros2 

    # Define the new dimensions
    width= int(w/ ds_factor)
    height = int(width * (h / w))

    # Resize the image
    resized_img = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    row_idx_start = int(height * row_crop_top)
    row_idx_end = int(height * row_crop_bottom)
    col_idx_start= int(width * col_crop_left)
    col_idx_end = int(width * col_crop_right)
    # mask_image = np.zeros(resized_img.shape[:2])
    resized_img_padded = np.zeros_like(resized_img)
    resized_img_padded[row_idx_start:row_idx_end, col_idx_start:col_idx_end, :] = resized_img[row_idx_start:row_idx_end, col_idx_start:col_idx_end, :]
    resized_img_gray = cv2.cvtColor(resized_img_padded, cv2.COLOR_BGR2GRAY)
    return resized_img_gray

class CameraFeedback():
    def __init__(self) -> None:
        super(CameraFeedback, self).__init__()
        self.camera_correction=np.array([0.,0.,0.])
        self.row_crop_pct_top = 0.0
        self.row_crop_pct_bot = 1.0
        self.col_crop_pct_left = 0.0
        self.col_crop_pct_right = 1.0

        self.ds_factor = 4 # Downsample factor

        self.x_dist_threshold = 2      # Thresholds to trigger feedback corrections
        self.y_dist_threshold = 2

        self.num_good_matches_threshold = 6
        self.correction_gain = 0.001         # metres of correction per pixel of (downsampled) image error
        self.max_correction_step = 0.005     # metres, clamp on a single correction step
        self.correction_increment = 0.0005   # metres, fixed step still used for the scale (z) correction

        # Hold a flagged timestep until the image error is corrected instead of moving on with it.
        self.max_sift_hold_steps = 50
        self.sift_hold_counter = 0
        self.sift_converged = True

        # A correction is applied from the median of a full measurement window and only once the
        # arm has settled on the previous one. Correcting every frame while the arm is still
        # moving feeds the lag back into the accumulator and the pose oscillates.
        self.sift_window = 5             # measurements per applied correction
        self.settle_tolerance = 0.002    # m, motion between two measurements that counts as settled
        self._sift_errors = []
        self._last_sift_pos = None
        self.camera_param_sub=self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 5)

        self.marker_pub = self.create_publisher(Marker, "/visualization_marker", 2)

        self.current_template_pub = self.create_publisher(Image, '/SIFT_corrections', 1)

        self.image_sub = self.create_subscription(Image, CAMERA_COLOR_TOPIC, self.image_callback, 5)

        self.cropped_img_pub = self.create_publisher(Image, '/modified_img', 1)

        self.bridge = CvBridge()
        
    def image_callback(self, msg):
            # Convert the ROS message to a OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            self.curr_image = cv_image

        except CvBridgeError as e:
            print(e)
    def camera_info_callback(self, camera_info):
        self.cx_cy_array = np.array([camera_info.k[2], camera_info.k[5]])    # Principal point offsets of your camera

    def sift_matching(self):
        if self.curr_image is None or not hasattr(self, "cx_cy_array"):
            return

        # self.resized_img_gray=image_process(self.ds_factor,  0, 1, 0, 1)
        self.resized_img_gray=image_process(self.curr_image, self.ds_factor,  self.row_crop_pct_top , self.row_crop_pct_bot, self.col_crop_pct_left, self.col_crop_pct_right)
        idx = self.time_index - 1

        # initiate SIFT detector
        sift = cv2.SIFT_create()

        # find the keypoints and descriptors with SIFT
        kp1, des1 = sift.detectAndCompute(self.loaded_img[idx], None)
        kp2, des2 = sift.detectAndCompute(self.resized_img_gray, None)
        if des1 is None or des2 is None:
            return

        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=100)

        flann = cv2.FlannBasedMatcher(index_params, search_params)

        # find matches by knn which calculates point distance in 128 dim
        try:
            matches = flann.knnMatch(des1, des2, k=2)
        except cv2.error as e:
            self.get_logger().warning(f"SIFT matching failed: {e}")
            return

        # store all the good matches as per Lowe's ratio test.
        good_feature = []
        for pair in matches:
            if len(pair) != 2:
                continue
            m, n = pair
            if m.distance < 0.7 * n.distance:
                good_feature.append(m)
            # translate keypoints back to full source template
        cx_cy_array_ds = self.cx_cy_array / self.ds_factor


        for k in kp1:
            k.pt = (k.pt[0] + self.col_crop_pct_left * self.resized_img_gray.shape[1] - cx_cy_array_ds[0], k.pt[1] + self.row_crop_pct_top * self.resized_img_gray.shape[0] - cx_cy_array_ds[1])
        for k in kp2:
            k.pt = (k.pt[0] - cx_cy_array_ds[0], k.pt[1] - cx_cy_array_ds[1])
        # print("after", kp1[0].pt)

        transform_correction = np.eye(4)
        transform_pixels = np.eye(2)
        apply_correction = False
        if len(good_feature) > self.num_good_matches_threshold:
            self._src_pts = np.float32([kp1[m.queryIdx].pt for m in good_feature]).reshape(-1, 1, 2)
            self._dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_feature]).reshape(-1, 1, 2)

            try:
                transform_pixels, inliers = cv2.estimateAffinePartial2D(self._src_pts, self._dst_pts)
            except cv2.error as e:
                self.get_logger().warning(f"SIFT transform estimation failed: {e}")
                return
            num_inliers = 0 if inliers is None else int(np.count_nonzero(inliers))
            if transform_pixels is not None:
                # print("transform", transform_pixels)
                scaling_factor = 1 - np.sqrt(np.linalg.det(transform_pixels[0:2, 0:2]))

                self._sift_errors.append(
                    (transform_pixels[0, 2], transform_pixels[1, 2], scaling_factor)
                )
                del self._sift_errors[:-self.sift_window]

                moved = 0.0 if self._last_sift_pos is None else float(
                    np.linalg.norm(np.asarray(self.curr_pos) - self._last_sift_pos)
                )
                self._last_sift_pos = np.asarray(self.curr_pos, dtype=float).copy()

                x_distance, y_distance, scaling_factor = np.median(self._sift_errors, axis=0)
                window_full = len(self._sift_errors) >= self.sift_window
                settled = moved <= self.settle_tolerance

                self.sift_converged = bool(
                    window_full
                    and abs(x_distance) <= self.x_dist_threshold
                    and abs(y_distance) <= self.y_dist_threshold
                )

                apply_correction = window_full and settled and not self.sift_converged
                self.get_logger().info(
                    f"SIFT t={idx}: {len(good_feature)} matches, {num_inliers} inliers, "
                    f"median dx={x_distance:.1f}px dy={y_distance:.1f}px over "
                    f"{len(self._sift_errors)}/{self.sift_window}, moved={moved * 1e3:.1f}mm"
                    f"{'' if apply_correction else ' -> gathering'}"
                )
                if apply_correction:
                    transform_correction = np.identity(4)

                    # Correction proportional to the measured pixel error: a fixed increment
                    # needs dozens of timesteps to close a large offset, so it never catches up
                    # on short feedback windows.
                    if abs(x_distance) > self.x_dist_threshold:
                        transform_correction[0, 3] = np.clip(
                            self.correction_gain * x_distance, -self.max_correction_step, self.max_correction_step
                        )
                    if abs(y_distance) > self.y_dist_threshold:
                        transform_correction[1, 3] = np.clip(
                            self.correction_gain * y_distance, -self.max_correction_step, self.max_correction_step
                        )

                    if abs(scaling_factor) > 0.05:
                        transform_correction[2, 3] = np.sign(scaling_factor) * self.correction_increment

                    # Fresh window for the next correction, measured after the arm has moved.
                    self._sift_errors.clear()

        for k in kp1:
            k.pt = (k.pt[0] + cx_cy_array_ds[0], k.pt[1] + cx_cy_array_ds[1])
        for k in kp2:
            k.pt = (k.pt[0] + cx_cy_array_ds[0], k.pt[1] + cx_cy_array_ds[1])

        if len(good_feature) > self.num_good_matches_threshold:
            try:
                M, mask = cv2.findHomography(self._src_pts, self._dst_pts, cv2.RANSAC, 5.0)
                matchesMask = mask.ravel().tolist()
                draw_params = dict(
                    matchColor=(0, 255, 0),
                    singlePointColor=None,
                    matchesMask=matchesMask,
                    flags=2,
                )
                padded_template = np.zeros_like(self.resized_img_gray)
                h, w = padded_template.shape
                row_idx_start = int(h * self.row_crop_pct_top)
                row_idx_end = int(h * self.row_crop_pct_bot)
                col_idx_start= int(w * self.col_crop_pct_left)
                col_idx_end = int(w * self.col_crop_pct_right)
                padded_template[row_idx_start:row_idx_end, col_idx_start:col_idx_end] = self.loaded_img[idx]
                self._annoted_image = cv2.drawMatches(padded_template, kp1, self.resized_img_gray, kp2, good_feature, None, **draw_params)
                loaded_image_msg = self.bridge.cv2_to_imgmsg(
                    self._annoted_image, encoding="bgr8"
                )
                self.current_template_pub.publish(loaded_image_msg)
            except Exception as e:
                print(e)

        if not apply_correction:
            return

        transform_base_2_cam = self.get_transform('panda_link0', 'camera_color_optical_frame')
        if transform_base_2_cam is None:
            return
        transform = transform_base_2_cam @ transform_correction @ np.linalg.inv(transform_base_2_cam)

        transform[2,3] = 0   # ignore z translation (in final transform/pose in base frame)
        self.camera_correction = self.camera_correction + transform[:3, 3]
        self.publish_correction_marker(transform)


    def publish_correction_marker(self, transform):
        marker = Marker()

        marker.header.frame_id = "panda_link0"
        marker.header.stamp = self.get_clock().now().to_msg()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 0
        marker.id = 0

        point_begin = Point()
        point_begin.x = self.curr_pos[0]
        point_begin.y = self.curr_pos[1]
        point_begin.z = self.curr_pos[2]
        point_end = Point()
        point_end.x = (self.curr_pos[0] + transform[0, 3]) * 1.2
        point_end.y = (self.curr_pos[1] + transform[1,3]) * 1.2
        point_end.z = self.curr_pos[2]

        marker.points = [point_begin, point_end]
        # Set the scale of the marker
        marker.scale.x = 0.005
        marker.scale.y = 0.01
        marker.scale.z = 0.0

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)
