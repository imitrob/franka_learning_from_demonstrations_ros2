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
        self.correction_increment = 0.0005
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

        # self.resized_img_gray=image_process(self.ds_factor,  0, 1, 0, 1)
        self.resized_img_gray=image_process(self.curr_image, self.ds_factor,  self.row_crop_pct_top , self.row_crop_pct_bot, self.col_crop_pct_left, self.col_crop_pct_right)
        # idx=np.argmin(np.linalg.norm(self.recorded_traj-(self.curr_pos).reshape(3,1),axis=0))
        idx = self.time_index - 1

        # initiate SIFT detector
        sift = cv2.SIFT_create()

        # find the keypoints and descriptors with SIFT
        kp1, des1 = sift.detectAndCompute(self.recorded_img[idx], None)
        kp2, des2 = sift.detectAndCompute(self.resized_img_gray, None)

        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=100)

        flann = cv2.FlannBasedMatcher(index_params, search_params)

        # find matches by knn which calculates point distance in 128 dim
        matches = flann.knnMatch(des1, des2, k=2)

        # store all the good matches as per Lowe's ratio test.
        good_feature = []
        for m, n in matches:
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
        if len(good_feature) > self.num_good_matches_threshold:
            self._src_pts = np.float32([kp1[m.queryIdx].pt for m in good_feature]).reshape(-1, 1, 2)
            self._dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_feature]).reshape(-1, 1, 2)

            transform_pixels, inliers = cv2.estimateAffinePartial2D(self._src_pts, self._dst_pts)
            # print("transform", transform_pixels)
            scaling_factor = 1 - np.sqrt(np.linalg.det(transform_pixels[0:2, 0:2]))


            x_distance = transform_pixels[0, 2]
            y_distance = transform_pixels[1, 2]

            transform_correction = np.identity(4)
            
            if abs(x_distance) > self.x_dist_threshold:
                transform_correction[0, 3] = np.sign(x_distance) * self.correction_increment
                # print("correcting x")
            if abs(y_distance) > self.y_dist_threshold:
                transform_correction[1, 3] = np.sign(y_distance) * self.correction_increment
                # print("correcting y")

            if abs(scaling_factor) > 0.05:
                transform_correction[2,3] = np.sign(scaling_factor) * self.correction_increment
                # print("correcting z")

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
                padded_template[row_idx_start:row_idx_end, col_idx_start:col_idx_end] = self.recorded_img[idx]
                self._annoted_image = cv2.drawMatches(padded_template, kp1, self.resized_img_gray, kp2, good_feature, None, **draw_params)
                recorded_image_msg = self.bridge.cv2_to_imgmsg(self._annoted_image)
                self.current_template_pub.publish(recorded_image_msg)  
            except Exception as e:
                print(e)

        transform_base_2_cam = self.get_transform_camera()
        transform = transform_base_2_cam @ transform_correction @ np.linalg.inv(transform_base_2_cam)

        transform[2,3] = 0   # ignore z translation (in final transform/pose in base frame)
        self.camera_correction = self.camera_correction + transform[:3, 3]
        self.publish_correction_marker(transform)


    def publish_correction_marker(self, transform):
        marker = Marker()

        marker.header.frame_id = "panda_link0"
        marker.header.stamp = self.get_clock().now()

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
        marker.scale.z = 0

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)
