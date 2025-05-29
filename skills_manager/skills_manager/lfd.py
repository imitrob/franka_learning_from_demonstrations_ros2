#%%
#!/usr/bin/env python
import time
import math
import numpy as np
import time
import tf2_ros
from skills_manager.camera_feedback import CameraFeedback, image_process
from geometry_msgs.msg import PoseStamped
from panda_control import Panda, SpinningRosNode
from skills_manager.feedback import Feedback
from skills_manager.insertion import Insertion
from skills_manager.transfom import Transform 
from panda_control.pose_transform_functions import position_2_array, pos_quat_2_pose_st, list_2_quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from skills_manager.ros_param_manager import get_remote_parameters
import trajectory_data

OPEN_GRIPPER_WIDTH = 0.08 # How much gripper opens [m]

from lfd_msgs.srv import SetTemplate
from std_srvs.srv import Trigger

class LfD(Panda, Feedback, Insertion, Transform, CameraFeedback, SpinningRosNode):
    def __init__(self):
        super(LfD, self).__init__()
        
        self.r= self.create_rate(20)
        
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.curr_image = None
        self.recorded_traj = None
        self.recorded_ori_wxyz = None

        self.end = False
        self.grip_open_width = OPEN_GRIPPER_WIDTH

        self.insertion_force_threshold = 6
        self.retry_counter = 0

        self.set_localizer_client = self.create_client(SetTemplate, 'set_localizer', callback_group=self.callback_group)
        self.active_localizer_client = self.create_client(Trigger, 'active_localizer', qos_profile=QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT), callback_group=self.callback_group)

        time.sleep(1)

    def traj_rec(self, trigger=0.005):
        self.set_stiffness(0,0,0,0,0,0,0)

        init_pos = self.curr_pos
        vel = 0 
        print("Move robot to start recording.", flush=True)
        while vel < trigger:
            vel = math.sqrt((self.curr_pos[0]-init_pos[0])**2 + (self.curr_pos[1]-init_pos[1])**2 + (self.curr_pos[2]-init_pos[2])**2)
        self.recorded_traj = self.curr_pos
        self.recorded_ori_wxyz = self.curr_ori_wxyz
        if self.gripper_width < self.grip_open_width * 0.9:
            self.grip_value = 0
        else:
            self.grip_value = self.grip_open_width
        self.recorded_gripper= self.grip_value
        self.recorded_img_feedback_flag = np.array([0])
        self.recorded_spiral_flag = np.array([0])
     
        resized_img_gray=image_process(self.curr_image, self.ds_factor,  self.row_crop_pct_top , self.row_crop_pct_bot,
                                        self.col_crop_pct_left, self.col_crop_pct_right)
        resized_img_msg = self.bridge.cv2_to_imgmsg(resized_img_gray)
        self.cropped_img_pub.publish(resized_img_msg)
        self.recorded_img = resized_img_gray.reshape((1, resized_img_gray.shape[0], resized_img_gray.shape[1]))

        print("Recording started. Press e to stop.")
        while not self.end:
            while(self.pause):
                self.r.sleep()               
            self.recorded_traj = np.c_[self.recorded_traj, self.curr_pos]
            self.recorded_ori_wxyz  = np.c_[self.recorded_ori_wxyz, self.curr_ori_wxyz]
            self.recorded_gripper = np.c_[self.recorded_gripper, self.grip_value]

            resized_img_gray=image_process(self.curr_image, self.ds_factor, self.row_crop_pct_top , self.row_crop_pct_bot,self.col_crop_pct_left, self.col_crop_pct_right)
            resized_img_msg = self.bridge.cv2_to_imgmsg(resized_img_gray)
            self.cropped_img_pub.publish(resized_img_msg)
            self.recorded_img = np.r_[self.recorded_img, resized_img_gray.reshape((1, resized_img_gray.shape[0], resized_img_gray.shape[1]))]
            self.recorded_img_feedback_flag = np.c_[self.recorded_img_feedback_flag, self.img_feedback_flag]
            self.recorded_spiral_flag = np.c_[self.recorded_spiral_flag, self.spiral_flag]

            self.r.sleep()

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"

        goal.pose.position.x = self.curr_pos[0]
        goal.pose.position.y = self.curr_pos[1]
        goal.pose.position.z = self.curr_pos[2]
        
        goal.pose.orientation.w = self.curr_ori_wxyz[0]
        goal.pose.orientation.x = self.curr_ori_wxyz[1]
        goal.pose.orientation.y = self.curr_ori_wxyz[2]
        goal.pose.orientation.z = self.curr_ori_wxyz[3]
        
        self.move_to_pose_with_stampedpose(goal)

        self.set_stiffness(self.K_pos, self.K_pos, self.K_pos, self.K_ori, self.K_ori, self.K_ori, 0)
        self.get_logger().info("Ending trajectory recording")

    def execute(self, retry_insertion_flag=0):
        self.spiralling_occured = False
        # print('entered execute')
        start = PoseStamped()

        quat_start = list_2_quaternion(self.recorded_ori_wxyz[:, 0])
        start = pos_quat_2_pose_st(self.recorded_traj[:, 0], quat_start)
         
        self.go_to_pose(start)
        self.set_stiffness(self.K_pos, self.K_pos, self.K_pos, self.K_ori, self.K_ori, self.K_ori, 0)

        self.time_index=0

        if self.recorded_gripper[0][0] < self.grip_open_width/2 and self.gripper_width > 0.9 * self.grip_open_width:
            print("closing gripper")
            self.grasp_gripper(self.recorded_gripper[0][self.time_index])
            time.sleep(0.1)
        if self.recorded_gripper[0][0] > self.grip_open_width/2:
            print("opening gripper")
            self.move_gripper(self.recorded_gripper[0][self.time_index])
            time.sleep(0.1)


        while self.time_index <( self.recorded_traj.shape[1]):
            quat_goal = list_2_quaternion(self.recorded_ori_wxyz[:, self.time_index])
            goal = pos_quat_2_pose_st(self.recorded_traj[:, self.time_index] + self.camera_correction, quat_goal)
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = 'panda_link0'
            ori_threshold = 0.3
            pos_threshold = 0.1
            
            self.correct()

            if (self.recorded_gripper[0][self.time_index]-self.recorded_gripper[0][max(0,self.time_index-1)]) < -self.grip_open_width/2:
                # print("closing gripper")
                self.grasp_gripper(self.recorded_gripper[0][self.time_index])
                time.sleep(0.1)

            if (self.recorded_gripper[0][self.time_index]-self.recorded_gripper[0][max(0,self.time_index-1)]) > self.grip_open_width/2:
                # print("open gripper")
                self.move_gripper(self.recorded_gripper[0][self.time_index])
                time.sleep(0.1)
    
            self.move_to_pose_with_stampedpose(goal)

            if self.recorded_img_feedback_flag[0, self.time_index]:
                self.sift_matching()
            
            if self.recorded_spiral_flag[0, self.time_index]:
                if self.force.z > 5:
                    spiral_success, offset_correction = self.spiral_search(goal)
                    self.spiralling_occured = True
                    if spiral_success:
                        self.recorded_traj[0, self.time_index:] += offset_correction[0]
                        self.recorded_traj[1, self.time_index:] += offset_correction[1]

            goal_pos_array = position_2_array(goal.pose.position)
            pos_2_goal_diff = np.linalg.norm(self.curr_pos-goal_pos_array)
  
            if pos_2_goal_diff <= self.attractor_distance_threshold:
                self.time_index=self.time_index + 1

            force_xy_plane = np.sqrt(self.force.x ** 2 + self.force.y ** 2)
            if retry_insertion_flag and force_xy_plane > self.insertion_force_threshold:
                # print("Camera correction", self.camera_correction)
                if self.retry_counter >= 3:
                    self.move_gripper(self.grip_open_width)
                    break
                self.go_to_pose(start)
                self.time_index = 0
                self.retry_counter = self.retry_counter + 1
            self.r.sleep()
            # Stop playback if at end of trajectory (some indices might be deleted by feedback)
            if self.time_index == self.recorded_traj.shape[1]-1:
                break

    def save(self, file='last'):
        np.savez(trajectory_data.package_path + '/trajectories/' + str(file) + '.npz',
                 traj=self.recorded_traj,
                 ori=self.recorded_ori_wxyz,
                 grip=self.recorded_gripper,
                 img=self.recorded_img, 
                 img_feedback_flag=self.recorded_img_feedback_flag,
                 spiral_flag=self.recorded_spiral_flag)
    
    def load(self, file='last'):
        data = np.load(trajectory_data.package_path + '/trajectories/' + str(file) + '.npz')
        self.recorded_traj = data['traj']
        self.recorded_ori_wxyz = data['ori']
        self.recorded_gripper = data['grip']
        self.recorded_img = data['img']
        self.recorded_img_feedback_flag = data['img_feedback_flag']
        self.recorded_spiral_flag = data['spiral_flag']
        if self.final_transform is not None:
            self.recorded_traj, self.recorded_ori_wxyz = self.transform_traj_ori(self.recorded_traj, self.recorded_ori_wxyz, self.final_transform)
        
        self.filename=str(file)

    def play_skill(self, name_skill, object_template_name, localize_box=True):
        if localize_box:
            if not self.set_localizer_client.wait_for_service(timeout_sec=5.0):
                raise Exception("Service not available after waiting")
            self.set_localizer_client.call(SetTemplate.Request(template_name=object_template_name))
            self.move_template_start()
            self.active_localizer_client.call(Trigger.Request())
            self.compute_final_transform() 
        else:
            pass # move to start?

        try:
            self.load(name_skill)
            print(f"Execution", flush=True)
            self.execute()
        except KeyboardInterrupt:
            pass

    def move_template_start(self):
        pose = get_remote_parameters(self, param_names=[
            "position_x", "position_y", "position_z", 
            "orientation_w", "orientation_x", "orientation_y", "orientation_z"],
            server="localizer_node")

        assert pose[2] > 0.02
        pos_array = pose[:3]
        quat_wxyz = np.quaternion(pose[3], pose[4], pose[5], pose[6])
        
        goal = pos_quat_2_pose_st(pos_array, quat_wxyz)
        goal.header.stamp = self.get_clock().now().to_msg()

        print(f"Move to start: x={goal.pose.position.x} y={goal.pose.position.y} y={goal.pose.position.z}", flush=True)
        self.go_to_pose_ik(goal)
        time.sleep(4.0) # could be deleted because go_to_pose_ik waits until finished
