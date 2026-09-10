#%%
#!/usr/bin/env python
import math
import os
import tempfile
import time
import quaternion
import numpy as np
import tf2_ros
from skills_manager.camera_feedback import CameraFeedback, image_process
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from lfd_msgs.srv import SetTemplate
from std_srvs.srv import Trigger
from std_msgs.msg import Int32
from panda_control import Panda, SpinningRosNode
from skills_manager.feedback import Feedback
from skills_manager.signalizer import Signalizator
from skills_manager.insertion import Insertion
from skills_manager.transfom import Transform
from panda_control.pose_transform_functions import position_2_array, pos_quat_2_pose_st, list_2_quaternion, invert_tf
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from skills_manager.ros_param_manager import get_remote_parameters
from copy import deepcopy
import spatialmath as sm
import trajectory_data
from trajectory_data.skill_visualizer import show_skill
from nocode_robot_programming.state_decision.utils import Filename

class SkillVis():
    def show(self, name_skill: str):
        show_skill(name_skill)

class LfD(Feedback, Panda, Insertion, Transform, CameraFeedback, SpinningRosNode, SkillVis):
    def __init__(self):
        super(LfD, self).__init__()
        self.freq = 10
        self.r = self.create_rate(self.freq)

        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.curr_image = None
        self.recorded_traj = None
        self.recorded_ori_wxyz = None
        self.loaded_traj = None
        self.loaded_ori_qxyz = None

        self.end = False
        self.filename = ""

        self.insertion_force_threshold = 6
        self.retry_counter = 0
        self.time_index = 0

        self.set_localizer_client = self.create_client(SetTemplate, 'set_localizer', callback_group=self.callback_group)
        self.active_localizer_client = self.create_client(Trigger, 'active_localizer', qos_profile=QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT), callback_group=self.callback_group)
        self.start_publishing_scene_call = self.create_client(Trigger, 'start_publishing_scene', qos_profile=QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT), callback_group=self.callback_group)
        self.stop_publishing_scene_call = self.create_client(Trigger, 'stop_publishing_scene', qos_profile=QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT), callback_group=self.callback_group)

        time.sleep(1)

        self.signalizer = Signalizator()

    @property
    def loaded_trajectory_len(self):
        return 0 if self.loaded_traj is None else self.loaded_traj.shape[1]

    @property
    def time_phase(self):
        if self.loaded_trajectory_len == 0: return 0

        return self.time_index / self.loaded_trajectory_len

    def traj_rec(self, trigger: float = 0.005, roll_redution_alpha: float = 0.4,
                 should_stop=None, on_phase=None, signalize: bool = True):
        """ Demonstrate a trajectory with either joystic, gestures, or kinesthetic teaching.
        Fills:
            self.recorded_traj
            self.recorded_ori_wxyz
            self.recorded_gripper
            self.recorded_img_feedback_flag
            self.recorded_spiral_flag
            ...

        Args:
            roll_reduction_alpha: float. When controlled externally (joystick/gestures), we let roll->0 as the user cannot control it.
        """
        requested_stop = should_stop or (lambda: False)
        def should_stop():
            self.check_motion()
            return requested_stop()
        if signalize:
            self.signalizer.signalize_ready_demonstration()
        if on_phase:
            on_phase("ready")

        # Old one-shot callers may carry a stale end flag. An action caller
        # supplies should_stop and deliberately treats an early finish as done.
        if on_phase is None:
            while self.end or self.pause:
                self.end = False
                self.pause = False
                time.sleep(0.1)
        elif self.end or should_stop():
            return False

        recording_started = False
        self.set_stiffness(0,0,0,0,0,0,0)
        try:
            init_pos = self.curr_pos
            vel = 0
            print("Move robot to start recording.", flush=True)
            while vel < trigger and not self.end and not should_stop():
                self.r.sleep()

                if self.is_applied_external_feedback(): # feedback changed and not kinesthetic teaching
                    time.sleep(0.1)
                    print("External control, setting stiffness!", flush=True)
                    self.set_stiffness(1000, 1000, 1000, 400, 400, 400, 0)
                    break
                vel = math.sqrt((self.curr_pos[0]-init_pos[0])**2 + (self.curr_pos[1]-init_pos[1])**2 + (self.curr_pos[2]-init_pos[2])**2)

            if self.end or should_stop():
                return False

            self.recorded_traj = self.curr_pos
            self.recorded_ori_wxyz = self.curr_ori_wxyz
            self.recorded_gripper= self.grip_value
            self.recorded_img_feedback_flag = np.array([0])
            self.recorded_spiral_flag = np.array([0])
            self.init_additional_flags()
            self.recorded_img = self.pub_rec_image()
            recording_started = True

            if signalize:
                self.signalizer.signalize_demonstration()
            if on_phase:
                on_phase("recording")
            print("Recording started. Press e to stop.")
            while not self.end and not should_stop():
                while(self.pause and not should_stop()):
                    print("Paused", flush=True)
                    time.sleep(0.5)
                if should_stop():
                    break
                t0 = time.perf_counter()
                self.recorded_traj = np.c_[self.recorded_traj, self.curr_pos]
                self.recorded_ori_wxyz  = np.c_[self.recorded_ori_wxyz, self.curr_ori_wxyz]
                self.recorded_gripper = np.c_[self.recorded_gripper, self.grip_value]
                self.recorded_img = np.r_[self.recorded_img, self.pub_rec_image()]

                self.recorded_img_feedback_flag = np.c_[self.recorded_img_feedback_flag, self.img_feedback_flag]
                self.recorded_spiral_flag = np.c_[self.recorded_spiral_flag, self.spiral_flag]

                cx, cy, cz, cw = self.curr_ori_xyzw
                q_curr = sm.UnitQuaternion([cw, cx, cy, cz])  # [w,x,y,z]
                goal = PoseStamped()

                trans_speed = 0.0
                if self.gesture_feedback is not None:
                    if np.linalg.norm(np.array(self.gesture_feedback) - np.array(self.curr_pos)) > 0.2:
                        print(f"too big step, safe quitting, would go to {self.gesture_feedback} from {self.curr_pos} in one step", )
                        continue
                    goal.pose.position = Point(
                        x=self.gesture_feedback[0],
                        y=self.gesture_feedback[1],
                        z=self.gesture_feedback[2],
                    )

                elif self.joystick_feedback is not None:
                    goal.pose.position = Point(
                        x=self.curr_pos[0] + self.joystick_feedback[0],
                        y=self.curr_pos[1] + self.joystick_feedback[1],
                        z=self.curr_pos[2] + self.joystick_feedback[2],
                    )
                    trans_speed = np.linalg.norm(self.joystick_feedback)

                else:
                    goal.pose.position = Point(
                        x=self.curr_pos[0],
                        y=self.curr_pos[1],
                        z=self.curr_pos[2],
                    )

            # Joystick increments (radians): Z (yaw), Y (pitch)
                dqz = sm.UnitQuaternion.Rz(self.rot_feedback[0])
                dqy = sm.UnitQuaternion.Ry(self.rot_feedback[1])

                q_pre = q_curr * dqz * dqy

                alpha_when_moving = 0.02
                alpha = alpha_when_moving + (roll_redution_alpha - alpha_when_moving) * np.exp(-4.0*trans_speed)
                q_goal = Transform.step_toward_roll(q_pre, alpha=alpha)

                qx, qy, qz, qw = q_goal.vec_xyzs  # returns (x,y,z,w)
                goal.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

                self.move_to_pose_with_stampedpose(goal)
                if self.feedback_gripper == "grasp":
                    print("closing gripper")

                    if not self.gripper_state.is_grasped:
                        self.grasp_gripper(0)
                    time.sleep(0.1)
                    self.feedback_gripper = ""

                if self.feedback_gripper == "open":
                    print("open gripper")
                    self.move_gripper(self.grip_open_width)
                    time.sleep(0.1)
                    self.feedback_gripper = ""

                self.update_additional_flags()
                if (time.perf_counter() - t0) * 0.8 > (1.0 / self.freq):
                    print(f"WARN: trajectory recording at {round(1.0 / (time.perf_counter() - t0))} samples per sec")
                self.r.sleep()
        finally:
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
            self.set_stiffness(self.K_pos, self.K_pos, self.K_pos,
                               self.K_ori, self.K_ori, self.K_ori, 0)
            self.get_logger().info("Ending trajectory recording")
            if signalize:
                self.signalizer.signalize_idle()
        return recording_started

    def save(self, file: str = 'last', overwrite: bool = True) -> bool:
        if self.recorded_traj is None or self.recorded_ori_wxyz is None:
            print("Cannot save, recording is empty", flush=True)
            return False

        if self.final_transform is not None:
            self.recorded_traj, self.recorded_ori_wxyz = self.transform_traj_ori(self.recorded_traj, self.recorded_ori_wxyz, invert_tf(self.final_transform))

        directory = os.path.join(trajectory_data.package_path, "trajectories")
        os.makedirs(directory, exist_ok=True)
        target = os.path.join(directory, f"{file}.npz")
        if os.path.exists(target) and not overwrite:
            raise FileExistsError(f"skill {file!r} already exists")

        temporary = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="wb", suffix=".npz", dir=directory, delete=False
            ) as stream:
                temporary = stream.name
                np.savez(stream,
                         traj=self.recorded_traj,
                         ori=self.recorded_ori_wxyz,
                         grip=self.recorded_gripper,
                         img=self.recorded_img,
                         img_feedback_flag=self.recorded_img_feedback_flag,
                         spiral_flag=self.recorded_spiral_flag)
            from trajectory_data.skill_part import SkillPart
            SkillPart(os.path.basename(temporary)).validate_archive(directory)
            os.replace(temporary, target)
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)
        return True

    def load(self, file='last'):
        data = np.load(trajectory_data.package_path + '/trajectories/' + str(file) + '.npz')
        self.loaded_traj = data['traj']
        self.loaded_ori_wxyz = data['ori']
        self.loaded_gripper = data['grip']
        self.loaded_img = data['img']
        self.loaded_img_feedback_flag = data['img_feedback_flag']
        self.loaded_spiral_flag = data['spiral_flag']
        if self.final_transform is not None:
            self.loaded_traj, self.loaded_ori_wxyz = self.transform_traj_ori(self.loaded_traj, self.loaded_ori_wxyz, self.final_transform)

        self.filename=str(file)

    def init_additional_flags(self):
        pass
    def update_additional_flags(self):
        pass

    def call_motion_service(self, client, request, *, localizing=False, timeout=30.0):
        """Wait on ROS without hiding cancellation; localizer poses run on this worker."""
        self.check_motion()
        previous = getattr(self, "_localization_future", None)
        if localizing and previous is not None and not previous.done():
            raise RuntimeError("Previous localization is still running")
        deadline = time.monotonic() + timeout
        with self._motion_lock:
            self._accept_localizer_goals = localizing
            self.external_call_msg = None
        try:
            future = client.call_async(request)
            if localizing:
                self._localization_future = future
            while True:
                self.check_motion()
                if time.monotonic() >= deadline:
                    self.fail_motion("Robot service timed out")
                with self._motion_lock:
                    pose = self.external_call_msg
                    self.external_call_msg = None
                if pose is not None:
                    # Corrections are tracking targets, not permission for a large jump.
                    self._validate_target(
                        [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
                        [pose.pose.orientation.x, pose.pose.orientation.y,
                         pose.pose.orientation.z, pose.pose.orientation.w])
                    self.go_to_pose_ik_quick(pose)
                if future.done():
                    return future.result()
                self.motion_sleep(0.01)
        finally:
            with self._motion_lock:
                self._accept_localizer_goals = False
                self.external_call_msg = None

    def localize(self, object_template_name: str = ""):
        if object_template_name == "":
            print("No given object_template_name", flush=True)
            return False

        if not self.set_localizer_client.wait_for_service(timeout_sec=5.0):
            raise Exception("Service not available after waiting")
        ret = self.call_motion_service(self.set_localizer_client, SetTemplate.Request(template_name=object_template_name))
        if not ret.success:
            print("Returned because localizer not succesful", flush=True)
            return False
        self.move_template_start()
        active = self.call_motion_service(self.active_localizer_client, Trigger.Request(), localizing=True)
        if active is None or not active.success:
            # The object is not where the template says it is: servoing produced
            # no delta, so the recorded trajectory would run against thin air.
            reason = "no response" if active is None else active.message
            print(f"Returned because localization failed: {reason}", flush=True)
            return False
        self.compute_final_transform()

    def play_skill(self, name_skill, object_template_name, localize_box=True):
        if localize_box:
            if not self.set_localizer_client.wait_for_service(timeout_sec=5.0):
                raise Exception("Service not available after waiting")
            ret = self.call_motion_service(self.set_localizer_client, SetTemplate.Request(template_name=object_template_name))
            if not ret.success:
                print("Returned because localizer not succesful", flush=True)
                return
            self.move_template_start()
            self.call_motion_service(self.active_localizer_client, Trigger.Request(), localizing=True)
            self.compute_final_transform()
        try:
            self.load(name_skill)
            print(f"Execution", flush=True)
            self.execute()
        except KeyboardInterrupt:
            print("Keyboard interrupted", flush=True)

    def move_template_start(self):
        pose = get_remote_parameters(self, param_names=[
            "position_x", "position_y", "position_z",
            "orientation_w", "orientation_x", "orientation_y", "orientation_z"],
            server="localizer_node")

        assert pose[2] > 0.02
        pos_array = pose[:3]
        quat_wxyz = quaternion.quaternion(pose[3], pose[4], pose[5], pose[6])

        goal = pos_quat_2_pose_st(pos_array, quat_wxyz)
        goal.header.stamp = self.get_clock().now().to_msg()

        print(f"Move to start: x={goal.pose.position.x} y={goal.pose.position.y} y={goal.pose.position.z}", flush=True)

        self.go_to_pose_ik(goal)

        if not np.allclose(self.curr_pos, pose[:3], atol=2e-3) or not np.allclose(self.curr_ori_wxyz, pose[3:], atol=2e-2):
            self.set_stiffness(2000,2000,2000,150,150,150,0)
            self.go_to_pose_ik(goal)
            self.set_stiffness(1000,1000,1000,80,80,80,0)

    # player
    def execute(self):
        self.signalizer.signalize_execution()
        self.player_init()
        while self.time_index <( self.loaded_trajectory_len):
            self.player_step()
        self.signalizer.signalize_idle()

    def gripper_step(self, target_gripper: float):
        if self.IS_OPEN(target_gripper) and not self.is_open() and self.gripper.read_once().is_grasped:
            # print(f"opening gripper: {self.IS_OPEN(target_gripper)} {self.is_open()}", flush=True)
            self.move_gripper(self.grip_open_width)
        if not self.IS_OPEN(target_gripper) and self.is_open() and not self.gripper.read_once().is_grasped:
            # print("closing gripper: ", flush=True)
            if not self.is_grasped():
                print("grasp started, wait for the grasp end... ", end="")
                self.grasp_gripper(0.0)
                print("grasp ended!", flush=True)


    def pub_rec_image(self):
        resized_img_gray=image_process(self.curr_image, self.ds_factor,  self.row_crop_pct_top , self.row_crop_pct_bot, self.col_crop_pct_left, self.col_crop_pct_right)

        resized_img_msg = self.bridge.cv2_to_imgmsg(resized_img_gray)
        resized_img_msg.header.frame_id = f"{self.time_index}|{self.filename}" # frame_id is set to timestep index
        self.cropped_img_pub.publish(resized_img_msg)

        return resized_img_gray.reshape((1, resized_img_gray.shape[0], resized_img_gray.shape[1]))

    def player_init(self):
        assert self.loaded_traj is not None, "Trajectory not loaded"

        print(f"Executing: {self.filename}", flush=True)
        # init states
        self.time_index=0
        self.end = False
        self.pause = False
        self.spiralling_occured = False
        self.camera_correction.fill(0)
        self.sift_hold_counter = 0
        self.sift_converged = True
        self._sift_errors.clear()
        self._last_sift_pos = None

        # init pose
        start = PoseStamped()
        quat_start = list_2_quaternion(self.loaded_ori_wxyz[:, 0])
        start = pos_quat_2_pose_st(self.loaded_traj[:, 0], quat_start)
        self.go_to_pose_ik(start)

        self.set_stiffness(self.K_pos, self.K_pos, self.K_pos, self.K_ori, self.K_ori, self.K_ori, 0)
        self.gripper_step(self.loaded_gripper[0][0])

        # init recording of new execution attempt
        self.recorded_traj = self.curr_pos
        self.recorded_ori_wxyz = self.curr_ori_wxyz
        self.recorded_gripper = self.grip_value
        self.recorded_img_feedback_flag = np.array([0])
        self.recorded_spiral_flag = np.array([0])
        self.recorded_img = self.pub_rec_image()

        return start

    def player_step(self):
        self.check_motion()
        assert self.loaded_traj is not None, "Trajectory not loaded"

        quat_goal = list_2_quaternion(self.loaded_ori_wxyz[:, self.time_index])
        goal = pos_quat_2_pose_st(self.loaded_traj[:, self.time_index] + self.camera_correction, quat_goal)
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'panda_link0'

        self.correct()
        self._validate_target(position_2_array(goal.pose.position),
                              [quat_goal.x, quat_goal.y, quat_goal.z, quat_goal.w])

        self.gripper_step(self.loaded_gripper[0][self.time_index])

        self.move_to_pose_with_stampedpose(goal)

        sift_step = bool(self.loaded_img_feedback_flag[0, self.time_index])
        if sift_step:
            self.sift_matching()

        if self.loaded_spiral_flag[0, self.time_index]:
            if self.force.z > 5:
                spiral_success, offset_correction = self.spiral_search(goal)
                self.spiralling_occured = True
                if spiral_success:
                    self.loaded_traj[0, self.time_index:] += offset_correction[0]
                    self.loaded_traj[1, self.time_index:] += offset_correction[1]

        goal_pos_array = position_2_array(goal.pose.position)
        pos_2_goal_diff = np.linalg.norm(self.curr_pos-goal_pos_array)

        # Hold a camera-feedback step until SIFT has closed the image error; otherwise the
        # trajectory walks on after a single small correction and never converges.
        holding = sift_step and not self.sift_converged and self.sift_hold_counter < self.max_sift_hold_steps
        if holding:
            self.sift_hold_counter = self.sift_hold_counter + 1
        elif self.safety_checker():
            self.sift_hold_counter = 0
            self.sift_converged = True
            self.time_index=self.time_index + 1

        force_xy_plane = np.sqrt(self.force.x ** 2 + self.force.y ** 2)
        if False and force_xy_plane > self.insertion_force_threshold:
            # print("Camera correction", self.camera_correction)
            if self.retry_counter >= 3:
                self.move_gripper(self.grip_open_width)

                return 'stop'

            self.go_to_pose(start) # PoseStamped
            self.time_index = 0
            self.retry_counter = self.retry_counter + 1
        self.motion_sleep(1.0 / self.freq)

        # save step sample
        self.recorded_traj = np.c_[self.recorded_traj, self.curr_pos]
        self.recorded_ori_wxyz  = np.c_[self.recorded_ori_wxyz, self.curr_ori_wxyz]
        self.recorded_gripper = np.c_[self.recorded_gripper, self.grip_value]

        self.recorded_img = np.r_[self.recorded_img, self.pub_rec_image()]
        self.recorded_img_feedback_flag = np.c_[self.recorded_img_feedback_flag, self.img_feedback_flag]
        self.recorded_spiral_flag = np.c_[self.recorded_spiral_flag, self.spiral_flag]

    def start_publishing_scene(self):
        self.start_publishing_scene_call.call(Trigger.Request())

    def stop_publishing_scene(self):
        self.stop_publishing_scene_call.call(Trigger.Request())


