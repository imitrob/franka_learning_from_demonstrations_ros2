
from dataclasses import dataclass
import pathlib, cv2, os, time, math
import numpy as np
from typing import Iterable, Tuple, List
from copy import deepcopy

from skills_manager.lfd import LfD
from panda_control.panda import robot_operation, MotionError
from skills_manager.risk_aware_lfd.risk_policy import *
from skills_manager.feedback import RiskAwareFeedback
import trajectory_data
from panda_control.pose_transform_functions import pos_quat_2_pose_st, list_2_quaternion

import rclpy
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from lfd_msgs.srv import StringService
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

import torch
from playsound import playsound
from threading import Thread

from nocode_robot_programming.state_decision.utils import Filename, get_session
from nocode_robot_programming.teaching.user_study_widget import choose_with_popup
from skills_manager.jupyter_widget_panel import JupyterWidgetPanel
from lfd_msgs.srv import SetTemplate

EXPECTED_TARGET_STATE_PUB_FREQ = 0.15 # sec

@dataclass
class Request():
    action: str = "play"
    timestep: int = 0
    task_name: str = ""
    valid_actions = {"play", "rec", "done"}

    def __post_init__(self):
        if self.action not in self.valid_actions:
            raise ValueError(f"Invalid action: '{self.action}'. Valid actions are: {self.valid_actions}")


class RALfD(JupyterWidgetPanel, RiskAwareFeedback, LfD):
    # Guard direct notebook calls as well as widget callbacks. Nested calls share ownership.
    home = robot_operation(LfD.home)
    traj_rec = robot_operation(LfD.traj_rec)
    move_template_start = robot_operation(LfD.move_template_start)


    def __init__(self, estimator_risk_policy: str = 'ContinueRiskPolicy', risk_patience: int = 2,
                 save_ds_window: bool = False):
        """
        Args:
            estimator_risk_policy (str): What to do when Risk Estimator detects risk
            human_risk_policy (str): What to do when Human signalizes risk
            risk_patience (int): How many risky samples next to each other to trigger risk. Defaulting to 2.
            save_ds_window (bool): When True, also save the short uncertain decision-state window
                (the last `window_size` samples before the branch) under the new label. Defaults to
                False, i.e. those samples are discarded rather than saved as a separate segment.
        """
        super(RALfD, self).__init__()
        self.create_subscription(String, "/target_state", self.target_state_callback, 5)

        self.risk_policy = eval(estimator_risk_policy)(risk_patience)
        self.save_ds_window = save_ds_window

        self.target_state = ""
        self.last_target_state = 0.0

        # self.haptic_buzz_pub = self.create_publisher(Float32, "/haptic_feedback", 5)

        self.retrain_client = self.create_client(StringService, 'state_decider_retrain', qos_profile=QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT), callback_group=self.callback_group)

        self.filename_obj: Filename | None = None

    @property
    def filename(self) -> str:
        if self.filename_obj is not None:
            return self.filename_obj.to_str()
        else:
            return ""

    @filename.setter
    def filename(self, value):
        self.filename_obj = Filename(value)


    def retrain(self, task_name: str):
        # self.retrain_client.call(StringService.Request(text=str(task_name)))
        self.end = False

        # call_async() on a service that hasn't been discovered yet returns a future
        # that never completes, and it does NOT reconnect when the server appears
        # later. So wait for the switcher first, looping so we recover once the user
        # starts it. Respect self.end / shutdown so we can still be aborted.
        warned = False
        while rclpy.ok() and not self.end and \
                not self.retrain_client.wait_for_service(timeout_sec=1.0):
            self.check_motion()
            if not warned:
                print("switcher not available",flush=True)
                print("Run switcher: ros2 run nocode_robot_programming switcher",flush=True)
                warned = True

        if self.end or not rclpy.ok():
            return None
        if warned:
            print("[training] Switcher detected, continuing.", flush=True)

        future = self.retrain_client.call_async(StringService.Request(text=str(task_name)))

        try:
            # Manual spin loop so we can check our own conditions
            while rclpy.ok() and not future.done() and not self.end:
                self.motion_sleep(0.1)

            # If we’re here because our end flag is set:
            if self.end:
                self.get_logger().info('Service call aborted because end flag is True.')
                # Optionally: future.cancel() – this only cancels waiting on the client side
                return None

            # If ROS is shutting down (Ctrl+C / shutdown):
            if not rclpy.ok():
                self.get_logger().info('Service call aborted because ROS is shutting down.')
                return None

            # Normal completion:
            if future.cancelled():
                self.get_logger().warn('Service call future was cancelled.')
                return None

            if future.exception() is not None:
                raise future.exception()

            return future.result()

        except KeyboardInterrupt:
            self.get_logger().info('KeyboardInterrupt: aborting service call.')
            # Optionally: future.cancel()
            return None

    def target_state_callback(self, msg):
        self.last_target_state = time.time()
        self.target_state = msg.data

    def skill_exists(self, name):
        return os.path.isfile(f"{trajectory_data.package_path}/trajectories/{get_session()}/{name}.npz")

    def init_additional_flags(self):
        self.recorded_risk_flag = np.array([0])
        self.recorded_safe_flag = np.array([0])
        self.recorded_novelty_flag = np.array([0])

    def update_additional_flags(self):
        self.recorded_risk_flag = np.c_[self.recorded_risk_flag, self.risk_flag]
        self.recorded_safe_flag = np.c_[self.recorded_safe_flag, self.safe_flag]
        self.recorded_novelty_flag = np.c_[self.recorded_novelty_flag, self.novelty_flag]

    # def get_observations(self) -> Tuple[torch.tensor, None, None, None, torch.tensor]:
    #     """Collect current observations as Enum list:
    #     [1. Image, 2. Risk, 3. Safe, 4. Novelty, 5. Time Phase]

    #     Returns:
    #         Tuple[torch.tensor, None, None, None, torch.tensor]
    #     """

    #     last_image_square = cv2.resize(self.pub_rec_image(), (64, 64), interpolation=cv2.INTER_AREA)
    #     last_image_square = last_image_square[np.newaxis, np.newaxis, :, :] # (x, 1, 64, 64)

    #     frame_number = np.array([self.time_phase]) # (x, 1)
    #     frame_number = frame_number[np.newaxis,:]

    #     return [
    #         torch.tensor(last_image_square, dtype=torch.float32).cuda(), # 1. Image
    #         None, # 2. Risk Label flag
    #         None, # 3. Safe Label flag
    #         None, # 4. Novelty Label flag
    #         torch.tensor(frame_number, dtype=torch.float32).cuda() # 5. Frame number normalized (0-1)
    #     ]

    @robot_operation
    def play_skill(self, name_skill, object_template_name, localize_box=True) -> List[Request]:
        
        self.request_log = []
        
        if not self.skill_exists(name_skill):
            print("Skill doesn't exist! returning")
            return []

        if localize_box and self.localize(object_template_name) is False:
            raise MotionError("Localization failed")

        # Track trials saved by this run so the last play can be undone
        self.last_saved_trial_names = saved_trial_exec_names = []
        try:
            request = Request(task_name = name_skill)
            while request.action != "done":
                print(f"New request: {request}")
                self.request_log.append(request)
                if request.action == "play":
                    self.show(request.task_name)
                    self.load(request.task_name)
                    new_request = self.execute()

                    # Proposal FIX: correct label around DS
                    # --------------------------
                    # anomaly was triggered with new potential DS, 
                    # we need to make sure here that DS window is saved with correct label
                    time.sleep(1.0)
                    ## TODO: I disabled double press to quit - there is some bug
                    # if self.end < 2: # When user press end more than once, we don't save
                    if new_request.action in ["play", "rec"]:
                        window_size = 10
                        # 1.) Save the clean execution-so-far under the ORIGINAL label,
                        #     excluding the uncertain decision-state window (last window_size samples).
                        save_name_ds = Filename(request.task_name, init_exec_trial=True)
                        self.save(save_name_ds.to_str(), split=slice(None, -window_size))
                        saved_trial_exec_names.append(save_name_ds.to_str())

                        # 2.) Optionally save the uncertain DS window under the NEW label.
                        #     It holds the last window_size samples before the branch, so it must be
                        #     positioned just before the decision state at [DS-window_size, DS-1].
                        #     The new label's offset is DS, so use offset = DS - window_size.
                        if self.save_ds_window:
                            nr = Filename(new_request.task_name)
                            save_name_initpart = Filename(
                                nr.task,
                                offset=nr.offset - window_size,
                                parent_offset=nr.parent_offset,
                                init_exec_trial=True,
                            )
                            self.save(save_name_initpart.to_str(), split=slice(-window_size, None))
                            saved_trial_exec_names.append(save_name_initpart.to_str())
                    else:
                        save_name = Filename(request.task_name, init_exec_trial=True)
                        self.save(save_name.to_str())

                        saved_trial_exec_names.append(save_name.to_str())
                        
                elif request.action == "rec":
                    self.traj_rec()
                    success = self.save(request.task_name)
                    if not success:
                        for file in saved_trial_exec_names:
                            self.remove(file)
                    new_request = Request(action="done")

                else: raise Exception("action not valid")

                request = new_request # update request

        except KeyboardInterrupt:
            print("Keyboard interrupted", flush=True)
        return self.request_log

    @robot_operation
    def execute(self) -> Request:
        ''' Has trajectory at self.loaded_traj, self.loaded_ori
        '''
        self.signalizer.signalize_execution()
        self.player_init()
        self.recorded_risk_flag = np.array([0])
        self.recorded_safe_flag = np.array([0])
        self.recorded_novelty_flag = np.array([0])

        while (time.time() - self.last_target_state) > EXPECTED_TARGET_STATE_PUB_FREQ:
            self.check_motion()
            print("waiting for target state", flush=True)
            self.motion_sleep(1.0)
            self.pub_rec_image()
            if self.end:
                break
        time.sleep(1.0)

        while self.time_index <( self.loaded_traj.shape[1]) and rclpy.ok() and not self.end:
            try:
                t0 = time.perf_counter()
                vel = 0
                init_pos = deepcopy(self.curr_pos)
                while(self.pause):
                    self.motion_sleep(1.0 / self.freq)
                    
                    if self.end or not rclpy.ok(): # user take control
                        save_name = Filename(self.filename_obj.task, 
                                             offset=self.time_index+self.filename_obj.offset,
                                             parent_offset=self.filename_obj.offset
                                            )
                        save_name.find_unique()

                        return Request(action="rec", timestep=self.time_index, task_name=save_name.to_str())
                    
                    vel = math.sqrt((self.curr_pos[0]-init_pos[0])**2 + (self.curr_pos[1]-init_pos[1])**2 + (self.curr_pos[2]-init_pos[2])**2)
                    if vel > 0.02:
                        print("Robot was moved manually! Continuing")
                        self.pause = False

                self.recorded_risk_flag = np.c_[self.recorded_risk_flag, self.risk_flag]
                self.recorded_safe_flag = np.c_[self.recorded_safe_flag, self.safe_flag]
                self.recorded_novelty_flag = np.c_[self.recorded_novelty_flag, self.novelty_flag]
                self.player_step()
                
                suggested_branch: str = self.target_state
                curr_branch: str = self.filename

                if suggested_branch.split("|")[0] == "manual_choose":
                    options = suggested_branch.split("|")[1:]

                    print("Waiting for user")
                    
                    suggested_branch = choose_with_popup(options, title="Select target skill part!")
                    assert isinstance(suggested_branch, str), f"suggested_branch should be string, it is: {type(suggested_branch)}"

                if suggested_branch == "continue":
                    suggested_branch = curr_branch

                # print(f"[step {self.time_index:3}]({int(round(1.0 / (time.perf_counter()-t0))):3}S/s) now: {curr_branch} -> suggested: {self.target_state}. anomaly: {suggested_branch == 'anomaly'}, {'SWITCHING!!!' if suggested_branch != curr_branch else ''}")
                ## TODO: Plotting current state interactively during execution
                if hasattr(self,'ui_progress_callback'):
                    self.ui_progress_callback(
                        self, 
                        step=self.time_index,
                        fps=1.0 / max(time.perf_counter() - t0, 1e-3),
                        curr_branch=curr_branch,
                        target_state=self.target_state,
                        suggested_branch=suggested_branch,
                        anomaly_flag=(suggested_branch == "anomaly"),
                    )

                if suggested_branch == "anomaly":
                    self.pause = True
                    continue

                if suggested_branch == "":
                    self.pause = True
                    print("State decider returning empty string, pausing. Most probably, you forgot to train the model.")
                    continue

                if suggested_branch != curr_branch:
                    return Request(action="play", timestep=self.time_index, task_name=suggested_branch)
            except rclpy.exceptions.ROSInterruptException or KeyboardInterrupt:
                print("manually interrupted", flush=True)
        
        if self.time_index < self.loaded_traj.shape[1]: # not finished ending
            save_name = Filename(self.filename_obj.task, 
                                 offset=self.time_index+self.filename_obj.offset, 
                                 parent_offset=self.filename_obj.offset
                                )
            save_name.find_unique()
            return Request(action="rec", timestep=self.time_index, task_name=save_name.to_str())

        self.signalizer.signalize_idle()
        return Request(action="done", timestep=self.time_index, task_name=self.filename)

    def go_to_time_index(self, time_index: int, linear: bool = False):
        """
        Args:
            time_index (int): target time index
            linear (bool, optional): Linear motion to time index, non-blocking. Defaults to False.
        """        
        quat_goal = list_2_quaternion(self.loaded_ori_wxyz[:, time_index])
        goal = pos_quat_2_pose_st(self.loaded_traj[:, time_index] + self.camera_correction, quat_goal)

        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'panda_link0'
        
        self.correct()

        self.handle_gripper(self.loaded_gripper[0][self.time_index])

        if linear:
            self.go_to_pose_ik(start)
            # self.go_to_pose(goal)
        else:
            self.move_to_pose_with_stampedpose(goal)
            # self.goal_pub.publish(goal)
        
    def delete_last_play(self) -> List[str]:
        """Delete the trial file(s) saved by the most recent play_skill run.

        Use when a finished play was saved but is invalid and must be discarded.
        Returns the list of trial names that were removed.
        """
        names = list(getattr(self, "last_saved_trial_names", []))
        if not names:
            print("[delete_last_play] no saved trial from the last play to delete.", flush=True)
            return []
        for file in names:
            self.remove(file)
        self.last_saved_trial_names = []
        print(f"[delete_last_play] removed {len(names)} trial file(s): {names}", flush=True)
        return names

    def remove(self, file: str):
        import os, pathlib
        if not pathlib.Path(f"{trajectory_data.package_path}/trajectories/{get_session()}/{file}.npz").is_file():
            print(f'file not exist {f"{trajectory_data.package_path}/trajectories/{get_session()}/{file}.npz"}')
            return 
        os.remove(f"{trajectory_data.package_path}/trajectories/{get_session()}/{file}.npz")
        print(f'to be removed {f"{trajectory_data.package_path}/trajectories/{get_session()}/{file}.npz"}')

    def save(self, file: str='last', tag: str = "", split: slice | None = None):
        """Saves Trajectory data to file
        If video_embedder and risk_estimator specified, then it is sampled and saved as video.

        Args:
            file (str | Iterable[str]): video file name to be saved.
        """        
        if (isinstance(file, Iterable) and not isinstance(file, str)):
            file = file[0]

        if len(self.recorded_img) < 3:
            print("no demonstration to save, returning")
            return False
        print(f"Saving Trajectory with len: {len(self.recorded_img)}")

        pathlib.Path(f"{trajectory_data.package_path}/trajectories/{get_session()}").mkdir(parents=True, exist_ok=True)

        if split is None:
            np.savez(f"{trajectory_data.package_path}/trajectories/{get_session()}/{file}.npz",
                    traj=              self.recorded_traj,
                    ori=               self.recorded_ori_wxyz,
                    grip=              self.recorded_gripper,
                    img=               self.recorded_img, 
                    img_feedback_flag= self.recorded_img_feedback_flag,
                    spiral_flag=       self.recorded_spiral_flag,
                    risk_flag=         self.recorded_risk_flag,
                    safe_flag=         self.recorded_safe_flag,
                    novelty_flag=      self.recorded_novelty_flag,
                    tag = tag,
                )
        else:
            assert isinstance(split, slice)
            if len(self.recorded_img[split,:,:]) == 0:
                print("short demonstration, saving only DS")
                return False
            
            np.savez(f"{trajectory_data.package_path}/trajectories/{get_session()}/{file}.npz",
                traj=              self.recorded_traj[:, split],
                ori=               self.recorded_ori_wxyz[:, split],
                grip=              self.recorded_gripper[:, split],
                img=               self.recorded_img[split,:,:], 
                img_feedback_flag= self.recorded_img_feedback_flag[:, split],
                spiral_flag=       self.recorded_spiral_flag[:, split],
                risk_flag=         self.recorded_risk_flag[:, split],
                safe_flag=         self.recorded_safe_flag[:, split],
                novelty_flag=      self.recorded_novelty_flag[:, split],
                tag = tag,
            )
        return True

            
    def load(self, file='last'):
        data = np.load(f"{trajectory_data.package_path}/trajectories/{get_session()}/{file}.npz")
        self.loaded_traj = data['traj']
        self.loaded_ori_wxyz = data['ori']
        self.loaded_gripper = data['grip']
        self.loaded_img = data['img']
        self.loaded_img_feedback_flag = data['img_feedback_flag']
        self.loaded_spiral_flag = data['spiral_flag']
        if 'tag' not in data.keys():
            self.tag = ""
        else:
            self.tag = str(data['tag'])

        if 'risk_flag' not in data.keys():
            self.loaded_risk_flag = np.zeros(data['grip'].shape)
        else:
            self.loaded_risk_flag = data['risk_flag']

        if 'safe_flag' not in data.keys():
            self.loaded_safe_flag = np.zeros(data['grip'].shape)
        else:
            self.loaded_safe_flag = data['safe_flag']

        if 'novelty_flag' not in data.keys():
            self.loaded_novelty_flag = np.zeros(data['grip'].shape)
        else:
            self.loaded_novelty_flag = data['novelty_flag']

        if self.final_transform is not None:
            self.loaded_traj, self.loaded_ori_wxyz = self.transform_traj_ori(self.loaded_traj, self.loaded_ori_wxyz, self.final_transform)
        
        self.filename_obj=Filename(str(file))

    # def vibrate(self):
    #     self.haptic_buzz_pub.publish(Float32(data=0.5))

    def communicate_risk_detected(self):
        music_thread = Thread(target=self.play_risk_detected)
        music_thread.start()
        # self.vibrate()

    def play_risk_detected(self):
        for _ in range(3):
            playsound('/usr/share/sounds/gnome/default/alerts/glass.ogg')
