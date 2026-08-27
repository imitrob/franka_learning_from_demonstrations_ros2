import numpy as np
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf_transformations
from panda_control.pose_transform_functions import orientation_2_quaternion, pose_st_2_transformation, position_2_array, pos_quat_2_pose_st, transformation_2_pose, transform_pose, list_2_quaternion, transform_pos_ori

from rclpy.duration import Duration 
from rclpy.time import Time
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import Point, Quaternion

from skills_manager.ros_param_manager import get_remote_parameters
import math
import spatialmath as sm
import quaternion

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
        quat_home_pose = quaternion.quaternion(orientation.w, orientation.x, orientation.y, orientation.z)
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
        try:
            stamped = self.tf_buffer.lookup_transform(
                source_frame, target_frame, Time(), timeout=Duration(seconds=0)
            )
        except tf2_ros.TransformException as e:
            self.get_logger().warning(
                f"Transform {source_frame} -> {target_frame} not available: {e}"
            )
            return None

        translation = stamped.transform.translation
        rotation = stamped.transform.rotation
        rp_tr = [translation.x, translation.y, translation.z]
        rp_rt = [rotation.x, rotation.y, rotation.z, rotation.w]
        transform = np.dot(tf_transformations.translation_matrix(rp_tr), tf_transformations.quaternion_matrix(rp_rt))
        return transform

    @staticmethod
    def step_toward_roll(q_pre, roll_target=math.pi, alpha=0.45):
        """
        Move a fraction (alpha in 0..1) from q_pre toward pose with the same yaw/pitch
        but roll = roll_target (body-frame ZYX convention). Returns a new quaternion.
        """
        # 1) Extract yaw/pitch from q_pre (ZYX)
        R = q_pre.R
        yaw   = math.atan2(R[1,0], R[0,0])
        pitch = math.atan2(-R[2,0], math.hypot(R[2,1], R[2,2]))

        # 2) Build target with desired roll
        q_target = sm.UnitQuaternion.Rz(yaw) * sm.UnitQuaternion.Ry(pitch) * sm.UnitQuaternion.Rx(roll_target)

        # 3) Relative rotation (current -> target), shortest-arc step of size alpha
        q_err = q_pre.inv() * q_target
        ex, ey, ez, ew = q_err.vec_xyzs  # (x,y,z,w)
        if ew < 0.0:  # enforce shortest arc
            ew, ex, ey, ez = -ew, -ex, -ey, -ez

        ew = max(-1.0, min(1.0, ew))
        angle = 2.0 * math.acos(ew)
        if angle < 1e-9:
            return q_pre

        s = math.sqrt(ex*ex + ey*ey + ez*ez)
        if s < 1e-12:
            axis = (1.0, 0.0, 0.0)
        else:
            axis = (ex/s, ey/s, ez/s)

        step = alpha * angle
        sh = math.sin(0.5 * step)
        q_step = sm.UnitQuaternion([math.cos(0.5 * step),
                                    axis[0]*sh, axis[1]*sh, axis[2]*sh])

        # 4) Apply small body-frame correction
        return q_pre * q_step
