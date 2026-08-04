#!/usr/bin/env python3
import yaml, time
import rclpy, os
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from sensor_msgs.msg import Image
from object_localization.localizer_sift import Localizer, detect_scene_features
import numpy as np

from lfd_msgs.srv import ComputeLocalization, SetTemplate, GetScene
from std_srvs.srv import Trigger
import tf_transformations

from cv_bridge import CvBridge
import object_localization
from object_localization.geometry import pixel_to_base, yaw_in_base
from object_localization.tf_utils import CustomTransformListener
from skills_manager.ros_param_manager import set_remote_parameters
from skills_manager.ros_utils import SpinningRosNode

from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
CAMERA_INFO_TOPIC = "/camera/color/camera_info"

# The scene is resolved in the robot base frame. "base" and "panda_link0" are the
# same physical frame here -- panda.py broadcasts panda_link0, while the gesture
# and pointing side (scene_marker_pub, a404.yaml) names it "base". We look up the
# TF under the name that is actually broadcast and publish under the name
# consumers expect.
ROBOT_BASE_TF_FRAME = "panda_link0"
SCENE_FRAME_ID = "base"
CAMERA_TF_FRAME = "camera_color_optical_frame"


class LocalizationService(CustomTransformListener, SpinningRosNode):
    def __init__(self) -> None:
        super(LocalizationService, self).__init__()

        self.declare_parameter("position_x",    0.0)
        self.declare_parameter("position_y",    0.0)
        self.declare_parameter("position_z",    0.0)
        self.declare_parameter("orientation_x", 0.0)
        self.declare_parameter("orientation_y", 0.0)
        self.declare_parameter("orientation_z", 0.0)
        self.declare_parameter("orientation_w", 0.0)
        self.declare_parameter("crop", 0.0)
        self.declare_parameter("depth", 0.0)

        # Height of the plane the camera ray is intersected with, in the base
        # frame. The table top is z=0, so that is the default. Note the measured
        # per-template `depth:` values sit ~79 mm short of the actual
        # camera-to-table range; that discrepancy is reported per object in
        # get_scene rather than silently absorbed into the position.
        self.declare_parameter("z_plane", 0.0)
        # Minimum RANSAC inliers before a match is trusted enough to publish.
        # Deliberately separate from localizer_sift.MIN_MATCH_COUNT, which the
        # working servo path depends on and which must not change.
        self.declare_parameter("min_inliers", 10)
        # How far an image stamp may sit beyond the newest TF sample before the
        # frame is dropped. A broadcaster at N Hz leaves a 1/N second hole after
        # each sample, and the camera's pipeline latency is shorter than that hole
        # at 10 Hz, so images land inside it and tf2 refuses to extrapolate
        # forward. Bounded rather than unlimited: the scene is only trusted near
        # home, where the arm is parked, so a few ms of staleness costs nothing --
        # but a second of it would silently localise against the wrong pose.
        self.declare_parameter("tf_future_tolerance", 0.05)

        self._rate = self.create_rate(5)
        # Template images and their SIFT descriptors are immutable, so build each
        # Localizer once instead of once per get_scene call.
        self._scene_localizers = {}
        self.camera_info_msg = None

        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, "/SIFT_localization", 10)
        self._publisher_counter = 0
        self._service = self.create_service(ComputeLocalization, 'compute_localization', self.handle_request, callback_group=self.callback_group)
        self._service_set_localizer = self.create_service(SetTemplate, 'set_localizer', self.set_localizer, callback_group=self.callback_group)

        self._service_get_scene = self.create_service(GetScene, 'compute_object_positions', self.get_scene, callback_group=self.callback_group)

        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 5)
        time.sleep(1)

    def _scene_localizer(self, name_template):
        """Cached Localizer for a template, or None if it has no params.yaml."""
        if name_template in self._scene_localizers:
            return self._scene_localizers[name_template]
        try:
            with open(f"{object_localization.package_path}/cfg/{name_template}/params.yaml") as f:
                tf_dict = yaml.safe_load(f)
        except FileNotFoundError:
            self._scene_localizers[name_template] = None
            return None

        template_path = f"{object_localization.package_path}/cfg/{name_template}/full_image.png"
        localizer = Localizer(template_path, tf_dict['crop'], tf_dict['depth'] * 0.001)
        self._scene_localizers[name_template] = localizer
        return localizer

    def _camera_in_base(self, stamp):
        """T_base<-camera at the given image stamp, or None if TF is not ready.

        Looked up at the image's own stamp rather than "latest": the camera is on
        the end effector, so using a newer transform than the picture would put
        objects wherever the arm has since moved to.
        """
        at_time = Time.from_msg(stamp)
        tolerance = Duration(
            seconds=float(self.get_parameter("tf_future_tolerance").value))
        base_hand_t, base_hand_r = self.lookup_relative_transform(
            ROBOT_BASE_TF_FRAME, "panda_hand", at_time=at_time,
            future_tolerance=tolerance)
        hand_cam_t, hand_cam_r = self.lookup_relative_transform(
            "panda_hand", CAMERA_TF_FRAME, at_time=at_time,
            future_tolerance=tolerance)
        if base_hand_t is None or hand_cam_t is None:
            return None

        def matrix(translation, rotation):
            return (
                tf_transformations.translation_matrix(
                    [translation.x, translation.y, translation.z])
                @ tf_transformations.quaternion_matrix(
                    [rotation.x, rotation.y, rotation.z, rotation.w])
            )

        return matrix(base_hand_t, base_hand_r) @ matrix(hand_cam_t, hand_cam_r)

    def get_scene(self, req, res):
        res.names = []
        res.pose = []

        if self.camera_info_msg is None:
            self.get_logger().warning("[scene] no camera_info yet, cannot un-project")
            return res

        all_templates = []
        for folder_items in os.walk(f"{object_localization.package_path}/cfg/"):
            all_templates = folder_items[1]
            break
        if len(all_templates) == 0:
            self.get_logger().warning("[scene] no templates found under cfg/")
            return res

        T_base_cam = self._camera_in_base(req.img.header.stamp)
        if T_base_cam is None:
            self.get_logger().warning("[scene] TF to the camera not available, skipping")
            return res

        K = np.array(self.camera_info_msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(self.camera_info_msg.d, dtype=np.float64)
        z_plane = float(self.get_parameter("z_plane").value)
        min_inliers = int(self.get_parameter("min_inliers").value)
        cv_image = self.bridge.imgmsg_to_cv2(req.img, "bgr8")
        # Detect on the frame once and share it: every template matches against
        # this same picture, and detecting was ~133 ms of the ~160 ms each
        # template cost. Turns N templates from N*160 ms into 133 + N*27 ms,
        # which is the difference between 0.7 Hz and 3 Hz at eight templates.
        scene_features = detect_scene_features(cv_image)

        res_names = []
        res_poses = []
        for name_template in sorted(all_templates):
            localizer = self._scene_localizer(name_template)
            if localizer is None:
                continue

            localizer.set_image(cv_image)
            localizer.set_camera_info(self.camera_info_msg)
            # annotate=False: the /SIFT_localization debug image is another
            # ~20 ms per template and only the servo path ever reads it.
            localizer.detect_points(scene_features=scene_features, annotate=False)

            match = localizer.measure_object()
            if match is None:
                self.get_logger().info(f"[scene] {name_template}: no match, skipped")
                continue
            if match.inliers < min_inliers:
                self.get_logger().info(
                    f"[scene] {name_template}: {match.inliers} inliers < {min_inliers}, skipped")
                continue

            position = pixel_to_base(K, D, T_base_cam, (match.u, match.v), z_plane)
            if position is None:
                self.get_logger().warning(
                    f"[scene] {name_template}: ray cannot reach z={z_plane}, skipped")
                continue

            quaternion = tf_transformations.quaternion_from_euler(
                0.0, 0.0, yaw_in_base(T_base_cam, match.yaw))

            # Diagnostic only -- the plane is authoritative for position. The SIFT
            # scale gives an independent range estimate; a persistent delta here
            # means the extrinsics or the template `depth:` values are off, which
            # would show up as a lateral offset rather than a wrong height.
            range_to_plane = float(np.linalg.norm(position - T_base_cam[0:3, 3]))
            range_from_scale = localizer.template_range() / match.scale
            self.get_logger().info(
                f"[scene] {name_template:14s} "
                f"t={range_to_plane:.3f} Z_scale={range_from_scale:.3f} "
                f"delta={1000.0 * (range_to_plane - range_from_scale):+.0f}mm "
                f"s={match.scale:.3f} yaw={np.rad2deg(match.yaw):+.1f}deg "
                f"inliers={match.inliers} -> "
                f"[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")

            res_names.append(name_template)
            res_poses.append(PoseStamped(
                header=Header(stamp=req.img.header.stamp, frame_id=SCENE_FRAME_ID),
                pose=Pose(
                    position=Point(x=float(position[0]), y=float(position[1]), z=float(position[2])),
                    orientation=Quaternion(x=float(quaternion[0]), y=float(quaternion[1]),
                                           z=float(quaternion[2]), w=float(quaternion[3])),
                ),
            ))

        res.names = res_names
        res.pose = res_poses
        return res

    def set_localizer(self, req, res):
        name_template = req.template_name

        try:
            with open(f"{object_localization.package_path}/cfg/{name_template}/params.yaml") as f:
                tf_dict = yaml.safe_load(f)
        except FileNotFoundError:
            print("TEMPLATE DOES NOT EXIST!!!", flush=True)
            print("TEMPLATE DOES NOT EXIST!!!", flush=True)
            print("TEMPLATE DOES NOT EXIST!!!", flush=True)
            print("TEMPLATE DOES NOT EXIST!!!", flush=True)
            print("TEMPLATE DOES NOT EXIST!!!", flush=True)
            print("TEMPLATE DOES NOT EXIST!!!", flush=True)
            print("TEMPLATE DOES NOT EXIST!!!", flush=True)
            print("TEMPLATE DOES NOT EXIST!!!", flush=True)
            res.success = False
            return res

        cropping = tf_dict['crop']
        depth = tf_dict['depth'] * 0.001
        
        set_remote_parameters(self, 
            ["crop", "depth", "position_x", "position_y", "position_z", "orientation_x", "orientation_y", "orientation_z", "orientation_w"],
            [tf_dict['crop'], tf_dict['depth'], tf_dict['position']["x"], tf_dict['position']["y"], tf_dict['position']["z"], tf_dict['orientation']["x"],
            tf_dict['orientation']["y"], tf_dict['orientation']["z"], tf_dict['orientation']["w"]]
            , server=self.get_name())
        
        template_path = f"{object_localization.package_path}/cfg/{name_template}/full_image.png"
        self._localizer = Localizer(template_path, cropping, depth)
        print(f"localizer set to {name_template}", flush=True)
        res.success = True
        return res

    def camera_info_callback(self, msg):
        self.camera_info_msg = msg
        

    def compute_localization_in_pixels(self, img: Image):
        cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        self._localizer.set_image(cv_image)
        self._localizer.set_camera_info(self.camera_info_msg)
        # try:
        self._localizer.detect_points()

        try: # if not successful -> annotated image not exist
            self._localizer.annoted_image()
        except Exception as e:
            print(e)
            print('Returning identity')
            return np.identity(4)
        
        tf_matrix = self._localizer.compute_full_tf_in_m()
        return tf_matrix
    
    def publish_annoted_image(self):
        ros_image = self.bridge.cv2_to_imgmsg(self._localizer.annoted_image(), "bgr8")
        self.image_publisher.publish(ros_image)

    def handle_request(self, req, response):
        tf_matrix = self.compute_localization_in_pixels(req.img)
        
        position = tf_matrix[0:3, 3]
        try:
            quaternion = tf_transformations.quaternion_from_matrix(tf_matrix[0:4, 0:4])
        except np.linalg.LinAlgError:
            quaternion = tf_transformations.quaternion_from_matrix(np.identity(4))

        quaternion = quaternion/np.linalg.norm(quaternion)
        # Publish pose
        pose = PoseStamped()
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation.w = quaternion[3]
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        try:
            self.publish_annoted_image()
        except:
            print("annotated image not exists", flush=True)

        response.pose=pose
        return response

def main():
    rclpy.init()
    simple_localizer_node = LocalizationService()

    print("Initialized", flush=True)
    try:
        while rclpy.ok():
            time.sleep(1.0)

    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
