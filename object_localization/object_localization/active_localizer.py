#!/usr/bin/env python3
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from panda_control import Panda, SpinningRosNode
from panda_control.pose_transform_functions import orientation_2_quaternion, pose_st_2_transformation, position_2_array, pos_quat_2_pose_st, transformation_2_pose, transform_pose, list_2_quaternion, transform_pos_ori, list_2_quaternion, pos_quat_2_pose_st

import tf_transformations
from lfd_msgs.srv import ComputeLocalization
import numpy as np
import time
from copy import deepcopy
from tf_transformations import euler_from_quaternion
from queue import Queue
import rclpy
from rclpy.duration import Duration

from skills_manager.ros_param_manager import get_remote_parameters
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped

from object_localization.tf_utils import CustomTransformListener

CAMERA_COLOR_TOPIC = '/camera/color/image_raw'

class ActiveLocalizerNode(CustomTransformListener, SpinningRosNode):
    def __init__(self) -> None:
        super(ActiveLocalizerNode, self).__init__()

        self._imgs = Queue(maxsize=1)
        self._img = None
        self._rate = self.create_rate(5)
        self.image_sub = self.create_subscription(Image, CAMERA_COLOR_TOPIC, self.image_callback, 5)


        self.compute_box_tf = self.create_client(ComputeLocalization, 'compute_localization', callback_group=self.callback_group)
        while not self.compute_box_tf.wait_for_service(timeout_sec=1.0):
            print('service ("compute_localization") not available, waiting again...')

        self._go_to = True
        self._window = Queue(maxsize=10)
        self._service = self.create_service(Trigger, 'active_localizer', self.handle_request, qos_profile=QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT), callback_group=self.callback_group)


        self.position_accuracy = 0.003
        self.orientation_accuracy=0.5 *(np.pi/180)
        self.timeout_counter_max = 50

        self.goal_pose_pub = self.create_publisher(PoseStamped, "/panda/goal_pose", 5)
        self.create_subscription(PoseStamped, "/panda/curr_pose", self.curr_pose_callback, 5)

        # self._prev_img = None
        self.img_last_rec = 0.0

    def curr_pose_callback(self, msg):
        self.curr_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.curr_ori_wxyz = [msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]

    def image_callback(self, img):
        # self._prev_img = deepcopy(self._img)
        self.img_last_rec = time.time()
        self._img = img
        
    def handle_request(self, req, res):
        print("Active localization started", flush=True)
        self._rate.sleep()
        self.timeout_counter = 0
        while rclpy.ok():
            if self._img is None: # or self._prev_img is None or list(self._img.data) == list(self._prev_img.data):
                self.get_logger().warning("No Image")
                self._rate.sleep()
                continue
            if (time.time() - self.img_last_rec) > 1.0:
                self.get_logger().warning("Img is not fresh")
                self._rate.sleep()
                continue
            position = self.curr_pos
            ori = list_2_quaternion(self.curr_ori_wxyz)
            home_pose = pos_quat_2_pose_st(position, ori)
            
            try:
                resp = self.compute_box_tf.call(request=ComputeLocalization.Request(img=self._img))
                box_tf = resp.pose
                ori = [
                    resp.pose.pose.orientation.x,
                    resp.pose.pose.orientation.y,
                    resp.pose.pose.orientation.z,
                    resp.pose.pose.orientation.w
                ]
                xy_yaw = [
                    resp.pose.pose.position.x, 
                    resp.pose.pose.position.y,
                    euler_from_quaternion(ori)[2]
                ]
            except Exception as e:
                print(e, flush=True)
                continue

            self._transformed_pose = self.transform(box_tf, home_pose)
 
            assert self._transformed_pose.pose.position.z > 0.1
            self.goal_pose_pub.publish(self._transformed_pose)
            self._rate.sleep()
            pos_error = np.linalg.norm(xy_yaw[:2])
            yaw_error = abs(xy_yaw[2])
            time.sleep(pos_error * 2) # Note: waits for the move to end, but not guaranteed
            time.sleep(0.1) # Note: waits for the move to end, but not guaranteed
            print("", flush=True)
            print("Localization step : ", self.timeout_counter, flush=True)
            print(f"position error {pos_error}, yaw error {yaw_error}", flush=True)
            print("", flush=True)
            if (pos_error < self.position_accuracy and yaw_error < self.orientation_accuracy) or self.timeout_counter >= self.timeout_counter_max:
                print(f"Localization finished! final error: {pos_error + yaw_error}", flush=True)
                return res
            self.timeout_counter = self.timeout_counter + 1

    def get_transform_camera(self):
        while True:
            try:
                translation1, rotation1 = self.lookup_relative_transform("panda_link0", "panda_hand")
                translation2, rotation2 = self.lookup_relative_transform("panda_hand", "camera_color_optical_frame")
                
                rp_tr1 = [translation1.x, translation1.y, translation1.z]
                rp_rt1 = [rotation1.x, rotation1.y, rotation1.z, rotation1.w]
                rp_tr2 = [translation2.x, translation2.y, translation2.z]
                rp_rt2 = [rotation2.x, rotation2.y, rotation2.z, rotation2.w]
                
                transform = np.dot(
                    tf_transformations.translation_matrix(rp_tr1),
                    tf_transformations.quaternion_matrix(rp_rt1),
                )
                transform = np.dot(
                    transform,
                    tf_transformations.translation_matrix(rp_tr2),
                )
                transform = np.dot(
                    transform,
                    tf_transformations.quaternion_matrix(rp_rt2),
                )
                return transform
                
            except Exception as e:
                time.sleep(0.3)
                print(f"Transform lookup failed: {e}. Retrying...", flush=True)

        
    
    def transform(self, transformation_pose, pose):
        transform_base_2_cam = self.get_transform_camera()

        # if transform box is not in camera frame, remove the base_2_cam transforms
        transform_box = pose_st_2_transformation(transformation_pose)
        transform = transform_base_2_cam @ transform_box @ np.linalg.inv(transform_base_2_cam)

        pose = transform_pose(pose, transform)
        pose_quat = orientation_2_quaternion(pose.pose.orientation)

        # Maintain orientation and only apply 'yaw' (rotation around EE z-axis)
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        new_magnitude = np.sqrt(pose_quat.x * pose_quat.x + pose_quat.y * pose_quat.y)
        pose_quat.x = pose_quat.x / new_magnitude
        pose_quat.y = pose_quat.y / new_magnitude
        pose.pose.orientation.x = pose_quat.x
        pose.pose.orientation.y = pose_quat.y

        home_EE_height = get_remote_parameters(self, ["position_z"], server="localizer_node")[0]
        assert home_EE_height > 0.1, f"Your template z-axis coords is below safety limit: {home_EE_height} < 0.1"
        pose.pose.position.z=home_EE_height  # Maintain same height
        return pose

def main():
    rclpy.init()
    rosnode = ActiveLocalizerNode()
    while rclpy.ok():
        time.sleep(1.0)

if __name__ == '__main__':
    main()