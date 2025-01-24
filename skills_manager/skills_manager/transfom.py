import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf_transformations
from panda_control.pose_transform_functions import orientation_2_quaternion, pose_st_2_transformation, position_2_array, pos_quat_2_pose_st, transformation_2_pose, transform_pose, list_2_quaternion, transform_pos_ori

from rclpy.duration import Duration 
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import Point, Quaternion

from skills_manager.ros_param_manager import get_remote_parameters

class Transform():
    def __init__(self):
        super(Transform, self).__init__()
        
        self.tf_buffer = Buffer()
        self._tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.final_transform = None
    def transform_traj_ori(self, traj, ori, transform):
        transformed_traj = np.empty_like(traj)
        transformed_ori = np.empty_like(ori)
        for i in range(traj.shape[1]):
            transformed_traj[:,i], transformed_ori[:,i] = transform_pos_ori(traj[:, i], ori[:, i], transform)
        return transformed_traj, transformed_ori

    def compute_final_transform(self):
        # Home pose of panda_EE frame
        pose = get_remote_parameters(self, ["position_x", "position_y", "position_z", "orientation_x", "orientation_y", "orientation_z", "orientation_w"], server="localizer_node")
        position = Point(x=pose[0], y=pose[1], z=pose[2])
        orientation = Quaternion(x=pose[3], y=pose[4], z=pose[5], w=pose[6])
        trans_home_pose = np.array([position.x, position.y, position.z])
        quat_home_pose = np.quaternion(orientation.w, orientation.x, orientation.y, orientation.z)
        self.home_pose = pos_quat_2_pose_st(trans_home_pose, quat_home_pose)
        transform_new = pose_st_2_transformation(self.curr_pose)
        home_pose_matrix = pose_st_2_transformation(self.home_pose)
        self.final_transform =  transform_new @ np.linalg.inv(home_pose_matrix)
        self.final_transform[2,0]=0
        self.final_transform[0,2]=0
        self.final_transform[2,1]=0
        self.final_transform[1,2]=0
        self.final_transform[2,2]=1
        self.final_transform[2,3]=0
        print("final transform", self.final_transform)
        return self.final_transform
    
    def get_transform(self, source_frame, target_frame):
        while True:
            try:
                now = self.get_clock().now()
                self._tf_listener.waitForTransform(source_frame, target_frame, now, Duration(seconds=4.0))
                rp_tr, rp_rt = self._tf_listener.lookupTransform(source_frame, target_frame, now)
                break
            except Exception as e:
                self.get_logger().warning(e)
        transform = np.dot(tf_transformations.translation_matrix(rp_tr), tf_transformations.quaternion_matrix(rp_rt))
        return transform
