#!/usr/bin/env python3
import yaml, time
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from object_localization.localizer_sift import Localizer
import numpy as np

from lfd_msgs.srv import ComputeLocalization, SetTemplate
import tf_transformations

from cv_bridge import CvBridge
import object_localization
from skills_manager.ros_param_manager import set_remote_parameters
from skills_manager.ros_utils import SpinningRosNode

from sensor_msgs.msg import CameraInfo

CAMERA_INFO_TOPIC = "/camera/color/camera_info"

class LocalizationService(SpinningRosNode):
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

        self._rate = self.create_rate(5)

        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, "/SIFT_localization", 10)
        self._publisher_counter = 0
        self._service = self.create_service(ComputeLocalization, 'compute_localization', self.handle_request, callback_group=self.callback_group)
        self._service_set_localizer = self.create_service(SetTemplate, 'set_localizer', self.set_localizer, callback_group=self.callback_group)

        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 5)
        time.sleep(1)

    def set_localizer(self, req, res):
        template_name = req.template_name

        with open(f"{object_localization.package_path}/cfg/{template_name}/params.yaml") as f:
            tf_dict = yaml.safe_load(f)

        cropping = tf_dict['crop']
        depth = tf_dict['depth'] * 0.001
        
        set_remote_parameters(self, 
            ["crop", "depth", "position_x", "position_y", "position_z", "orientation_x", "orientation_y", "orientation_z", "orientation_w"],
            [tf_dict['crop'], tf_dict['depth'], tf_dict['position']["x"], tf_dict['position']["y"], tf_dict['position']["z"], tf_dict['orientation']["x"],
            tf_dict['orientation']["y"], tf_dict['orientation']["z"], tf_dict['orientation']["w"]]
            , server=self.get_name())
        
        template_path = f"{object_localization.package_path}/cfg/{template_name}/full_image.png"
        self._localizer = Localizer(template_path, cropping, depth)
        print(f"localizer set to {template_name}", flush=True)
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
