#%%
#!/usr/bin/env python
import time, math
import quaternion
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import rclpy
import threading
import fcntl
from contextlib import contextmanager
from functools import wraps
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# from franka_gripper.msg import GraspActionGoal, HomingActionGoal, StopActionGoal, MoveActionGoal
from panda_control.pose_transform_functions import  pos_quat_2_pose_st, list_2_quaternion, pose_2_transformation, interpolate_poses, q_norm, q_angle, q_slerp, build_quat_seq, min_angle_condition, step_slerp 
from spatialmath import SE3 #pip install spatialmath-python
from spatialmath.base import q2r
import roboticstoolbox as rtb #pip install roboticstoolbox-python

from skills_manager.ros_param_manager import set_remote_parameters, get_remote_parameters, declare_parameter_slider
from skills_manager.ros_utils import SpinningRosNode

import panda_py
from panda_py.libfranka import Gripper
from panda_py import controllers
import numpy as np

# Panda hostname/IP and Desk login information of your robot
HOSTNAME = "192.168.88.140"
username = 'admin'
password = '123456789'

UPDATE_THREAD_INTERVAL = 1.0 # s
# TF must be dense enough that tf2 can answer for an image stamp: the camera's
# stamps are 25-50 ms old here and tf2 does not extrapolate forward, so a 1 s
# gap makes object_localization's get_scene skip most frames.
TF_BROADCAST_INTERVAL = 0.01 # s
OPEN_GRIPPER_WIDTH = 0.06 # How much gripper opens [m]
HIGH_POINT_DIFFERENCE = 0.1 # m
HIGH_ORI_DIFFERENCE = 0.1  # radians; tracking tolerance is half this limit
TRACKING_TIMEOUT = 5.0  # seconds without reaching the current waypoint
STOP_TIMEOUT = 2.0  # controller acknowledgement deadline
JOINT_NAMES = [f"panda_joint{i}" for i in range(1, 8)] + [
    "panda_finger_joint1", "panda_finger_joint2"]

from typing import Iterable
# panda-py is chatty, activate information log level
import logging
logging.basicConfig(level=logging.WARNING)
from copy import deepcopy
from panda_control.home_pose import HOME_POSE

### SUPER FAST STIFFNESS SETTING - NO ROS PARAM SET (cannot be changed it remotely)
DIRECT_STIFFNESS_OPTION = True

### End-effector payload (Panda Hand ~0.7 kg + RealSense D455 ~0.3 kg).
### libfranka splits the flange payload into m_ee (the end effector the
### system already knows about, e.g. the Franka Hand configured in Desk) and
### m_load (set via set_load). Gravity is compensated against
### m_total = m_ee + m_load. So set_load must declare ONLY the mass beyond the
### hand (the camera + mount) -- declaring the full total double-counts the
### hand and over-compensates gravity (arm rises at zero stiffness).
TOTAL_PAYLOAD_MASS = 1.05  # kg, full flange payload (hand + camera + mount)
LOAD_F_X_CLOAD = [-0.01, 0.0, 0.03]  # m, flange->load COM in the flange frame
LOAD_INERTIA = [0.001, 0.0, 0.0,
                0.0, 0.0025, 0.0,
                0.0, 0.0, 0.0017]  # kg*m^2, row-major 3x3
LOAD_MASS = False

class MotionCanceled(RuntimeError):
    pass


class MotionError(RuntimeError):
    pass


def robot_operation(method):
    @wraps(method)
    def run(self, *args, **kwargs):
        with self.robot_operation():
            return method(self, *args, **kwargs)
    return run


class Panda():
    def __init__(self,
                 K_pos: int = 1000, # Default Positional stiffness
                 K_ori: int = 30, # Default Orientation stiffness
                 K_ns: int = 0, # Default Nullspace stiffness
                 ):
        super(Panda, self).__init__()
        # ponytail: local Linux process ownership; remote clients must use the server.
        self._owner_file = open(f"/tmp/franka-{HOSTNAME}.lock", "a")
        try:
            fcntl.flock(self._owner_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self._owner_file.close()
            raise RuntimeError("Robot already owned by another demo process") from None
        self._init_motion_state()
        self.K_pos = K_pos
        self.K_ori = K_ori
        self.K_ns= K_ns

        self.curr_pos_goal=None
        self.curr_ori_goal_wxyz=None
        self.goal_pose=None
        self.attractor_distance_threshold=0.05
        self.grip_open_width = OPEN_GRIPPER_WIDTH
        self.safety_check=True
         
        self.translational_stiffness_X = self.K_pos
        self.translational_stiffness_Y = self.K_pos
        self.translational_stiffness_Z = self.K_pos
        self.rotational_stiffness_X = self.K_ori
        self.rotational_stiffness_Y = self.K_ori
        self.rotational_stiffness_Z = self.K_ori
        self.nullspace_stiffness = self.K_ns
        if not DIRECT_STIFFNESS_OPTION:
            declare_parameter_slider(self, "translational_stiffness_X", self.translational_stiffness_X, from_value=0, to_value=4000, step=1)
            declare_parameter_slider(self, "translational_stiffness_Y", self.translational_stiffness_Y, from_value=0, to_value=4000, step=1)
            declare_parameter_slider(self, "translational_stiffness_Z", self.translational_stiffness_Z, from_value=0, to_value=4000, step=1)
            declare_parameter_slider(self, "rotational_stiffness_X", self.rotational_stiffness_X, from_value=0, to_value=4000, step=1)
            declare_parameter_slider(self, "rotational_stiffness_Y", self.rotational_stiffness_Y, from_value=0, to_value=4000, step=1)
            declare_parameter_slider(self, "rotational_stiffness_Z", self.rotational_stiffness_Z, from_value=0, to_value=4000, step=1)
            declare_parameter_slider(self, "nullspace_stiffness", self.nullspace_stiffness, from_value=0, to_value=4000, step=1)

        self.desk = panda_py.Desk(HOSTNAME, username, password)
        self.desk.unlock()
        self.desk.activate_fci()

        self.panda = panda_py.Panda(HOSTNAME)
        self.panda.disable_logging()

        # Configure the end-effector load so the Cartesian impedance controller
        # compensates the payload's gravity. Must be done while idle (no motion
        # running yet), which is the case here in __init__ before ctrl_node starts.
        # The system already accounts for the configured end effector (m_ee, e.g.
        # the Franka Hand), so declare only the remaining mass to avoid
        # double-counting it (which over-compensates and lifts the arm).
        if LOAD_MASS:
            m_ee = self.panda.get_state().m_ee
            load_mass = max(TOTAL_PAYLOAD_MASS - m_ee, 0.0)
            self.panda.get_robot().set_load(load_mass, LOAD_F_X_CLOAD, LOAD_INERTIA)
            print(f"[panda] set_load: m_ee={m_ee:.3f} kg, m_load={load_mass:.3f} kg, "
              f"target m_total={TOTAL_PAYLOAD_MASS:.3f} kg", flush=True)

        self.gripper = Gripper(HOSTNAME)
        self.goal_position = None # Set (x,y,z) attractor
        self.goal_orientation = None # Set (1.0,0.0,0.0,0.0) attractor ori xyzw https://jeanelsner.github.io/panda-py/panda_py.html#panda_py.Panda.move_to_pose
        self.goal_q_nullspace = None

        self.break_control_requested = threading.Event() # e.g. if stiffness changed request
        self.break_control_done = threading.Event()

        self.create_subscription(PoseStamped, "/panda/goal_pose", self.external_call, 5)
        self.curr_pose_pub = self.create_publisher(PoseStamped, "/panda/curr_pose", 5)
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 5)

        self.tf_broadcaster = TransformBroadcaster(self)
        time.sleep(1)

        self.external_call_msg = None


    def _init_motion_state(self):
        self._motion_lock = threading.RLock()
        self._operation_lock = threading.RLock()
        self._operation_depth = 0
        self._motion_cancel = threading.Event()
        self._hold_done = threading.Event()
        self._controller_ready = threading.Event()
        self._motion_fault = ""
        self._tracking_since = None
        self._accept_localizer_goals = False
        self.external_call_msg = None
        self.tracking_timeout = TRACKING_TIMEOUT

    def begin_motion(self):
        with self._motion_lock:
            if not self._controller_ready.is_set():
                raise MotionError("Controller unavailable")
            if self._motion_cancel.is_set() and not self._hold_done.is_set():
                raise MotionError("Controller has not acknowledged stop")
            self._motion_fault = ""
            self._motion_cancel.clear()
            self._tracking_since = None

    @contextmanager
    def robot_operation(self):
        if not self._operation_lock.acquire(blocking=False):
            raise MotionError("Robot operation is already active")
        outer = self._operation_depth == 0
        try:
            if outer:
                self.begin_motion()
            self._operation_depth += 1
            try:
                yield
            except BaseException:
                if outer:
                    self.stop()
                    self.stop_gripper()
                    self.wait_for_hold()
                raise
            finally:
                self._operation_depth -= 1
        finally:
            self._operation_lock.release()

    def check_motion(self):
        if self._motion_fault:
            raise MotionError(self._motion_fault)
        if self._motion_cancel.is_set() or not rclpy.ok():
            raise MotionCanceled("Motion canceled")

    def motion_sleep(self, seconds):
        self._motion_cancel.wait(seconds)
        self.check_motion()

    def fail_motion(self, reason):
        with self._motion_lock:
            self._motion_fault = reason
            self.stop()
        raise MotionError(reason)

    def wait_for_hold(self):
        if not self._hold_done.wait(STOP_TIMEOUT):
            raise MotionError("Controller has not acknowledged stop; robot unavailable")

    def check_tracking(self, reached):
        self.check_motion()
        if reached:
            self._tracking_since = None
        elif self._tracking_since is None:
            self._tracking_since = time.monotonic()
        elif time.monotonic() - self._tracking_since >= self.tracking_timeout:
            self.fail_motion("Tracking timed out")
        return reached

    def has_realtime_kernel(self):
        return panda_py.libfranka.has_realtime_kernel()

    def is_grasped(self) -> bool:
        return self.gripper_state.is_grasped 

    def IS_OPEN(self, value: float):
        return float(value) > self.grip_open_width / 2.0

    def is_open(self):
        return not self.gripper_state.is_grasped
        
    def external_call(self, msg):
        with self._motion_lock:
            if self._accept_localizer_goals and not self._motion_cancel.is_set():
                self.external_call_msg = deepcopy(msg)
            else:
                self.get_logger().warning("Pose rejected: no active localization operation")

    def move_to_pose_with_stampedpose(self, pose: PoseStamped):
        self.move_to_pose(
            position=(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
            orientation=(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w),
            speed_factor=0.2,
        )
        self.ee_pos_goal_callback(pose)
        
    def ee_pos_goal_callback(self, goal_conf):
        self.goal_pose = goal_conf
        self.curr_pos_goal = np.array([goal_conf.pose.position.x, goal_conf.pose.position.y, goal_conf.pose.position.z])
        self.curr_ori_goal_wxyz = np.array([goal_conf.pose.orientation.w, goal_conf.pose.orientation.x, goal_conf.pose.orientation.y, goal_conf.pose.orientation.z])
        self.safety_checker()
        
    def move_gripper(self, width: float):
        self.move(width, speed=0.05)

    def open(self):
        self.move_gripper(self.grip_open_width)

    def grasp_gripper(self, width):
        self.check_motion()
        self.gripper.stop()
        self.gripper.grasp(width=width, speed=0.05, force=50, epsilon_inner=0.055, epsilon_outer=0.055)

    def home(self,
             height=HOME_POSE.position[2],
             front_offset=HOME_POSE.position[0],
             side_offset=HOME_POSE.position[1]):
        self.check_motion()
        # Cartesian interpolation keeps homing interruptible, including large rotations.
        pos_array = np.array([front_offset, side_offset, height])
        quat = quaternion.quaternion(*HOME_POSE.orientation_wxyz)
        goal = pos_quat_2_pose_st(pos_array, quat)
        goal.header.stamp = self.get_clock().now().to_msg()

        self.go_to_pose_ik(goal)
        self.print_home_error(pos_array, HOME_POSE.orientation_wxyz)

    def print_home_error(self, goal_pos, goal_ori_wxyz):
        """How far homing actually ended from the home pose. The impedance
        controller settles wherever its attractor balances gravity and
        friction, so this is never exactly zero -- print it to see the drift."""
        pos_err = np.array(self.curr_pos) - np.array(goal_pos)
        ang_err = q_angle(q_norm(self.curr_ori_wxyz), q_norm(goal_ori_wxyz))
        print(f"[home] position error: "
              f"x={pos_err[0]*1000:+.1f} y={pos_err[1]*1000:+.1f} z={pos_err[2]*1000:+.1f} mm "
              f"(norm {np.linalg.norm(pos_err)*1000:.1f} mm), "
              f"orientation error: {math.degrees(ang_err):.1f} deg", flush=True)

    def stop(self):
        with self._motion_lock:
            if not self._motion_cancel.is_set():
                self._hold_done.clear()
            self._motion_cancel.set()
            self.goal_position = None
            self.goal_orientation = None
            self.external_call_msg = None

    def home_gripper(self):
        self.gripper.homing()
        # self.homing_pub.publish(self.home_command)

    def stop_gripper(self):
        self.gripper.stop()
        # self.stop_pub.publish(self.stop_command)  

    def set_configuration(self,joint):
        joint_des=Float32MultiArray()
        joint_des.data= np.array(joint).astype(np.float32).tolist()
        # self.configuration_pub.publish(joint_des)
        self.goal_q_nullspace = tuple(joint)
    def set_stiffness(self, k_t1: int, k_t2: int, k_t3: int,k_r1: int,k_r2: int, k_r3: int, k_ns: int):
        
        if DIRECT_STIFFNESS_OPTION:
            k_t1, k_t2, k_t3 ,k_r1, k_r2, k_r3, k_ns = int(k_t1), int(k_t2), int(k_t3), int(k_r1), int(k_r2), int(k_r3), int(k_ns)
            
            self.translational_stiffness_X, self.translational_stiffness_Y, self.translational_stiffness_Z, self.rotational_stiffness_X, self.rotational_stiffness_Y, self.rotational_stiffness_Z, self.nullspace_stiffness = k_t1, k_t2, k_t3 ,k_r1, k_r2, k_r3, k_ns
            self.restart_control()
            
        else:
            set_remote_parameters(self, [
                "translational_stiffness_X", "translational_stiffness_Y", "translational_stiffness_Z",
                "rotational_stiffness_X", "rotational_stiffness_Y", "rotational_stiffness_Z", "nullspace_stiffness"
                ], [k_t1, k_t2, k_t3, k_r1, k_r2, k_r3, k_ns], server=self.get_name())
            self.restart_control()
                        


    # control robot to desired goal position
    def go_to_pose(self, goal_pose: PoseStamped, interp_dist=0.01, interp_dist_polar=0.01): 
        # the goal pose should be of type PoseStamped. E.g. goal_pose=PoseStampled()
        r = self.create_rate(100)
        
        poses=  interpolate_poses(self.curr_pose, goal_pose, interp_dist, interp_dist_polar)
        for pose in poses:
            
            self.move_to_pose_with_stampedpose(pose)
            self.motion_sleep(0.01)
        self.move_to_pose_with_stampedpose(goal_pose)    
        self.motion_sleep(0.2)
    
        # control robot to desired goal position

    def go_to_pose_ik_quick(self, goal_pose: PoseStamped, goal_configuration=None, interp_dist=0.002, interp_dist_joint=0.004):
        r = self.create_rate(200)
        self.move_to_pose_with_stampedpose(self.curr_pose)
        
        self.set_configuration(self.curr_joint)
        
        robot = rtb.models.Panda()
        position_start = self.curr_pos
        joint_start = np.array(self.curr_joint)
        goal_array = np.array([goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z])

        # interpolate from start to goal with attractor distance of approx 1 cm
        dist = np.sqrt(np.sum(np.subtract(position_start, goal_array)**2, axis=0))
        
        step_num_lin = math.floor(dist / interp_dist)

        # Orientation slerp endpoints. The quick variant used to command the goal
        # orientation on every step (an instant orientation jump that triggers
        # reflex errors on large rotations); interpolate the orientation instead.
        q_start_wxyz = q_norm([self.curr_pose.pose.orientation.w,
                               self.curr_pose.pose.orientation.x,
                               self.curr_pose.pose.orientation.y,
                               self.curr_pose.pose.orientation.z])
        q_goal_wxyz = q_norm([goal_pose.pose.orientation.w,
                              goal_pose.pose.orientation.x,
                              goal_pose.pose.orientation.y,
                              goal_pose.pose.orientation.z])
        max_ori_step = HIGH_ORI_DIFFERENCE / 4  # leave room for tracking lag
        step_num_ori = max(1, math.ceil(q_angle(q_start_wxyz, q_goal_wxyz) / max_ori_step))
        if goal_configuration is None:
            quaternion_array = np.array([goal_pose.pose.orientation.w, goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z]) 
            # normalize quaternion
            quaternion_array = quaternion_array / np.linalg.norm(quaternion_array)
            # Convert quaternion to rotation matrix
            rotation_matrix = q2r(quaternion_array)

            T = SE3.Rt(rotation_matrix, goal_array)

            # Solve inverse kinematics, try 5 times
            for i in range(5):
                # sol = robot.ikine_LM(T, q0=joint_start)
                sol = robot.ikine_LM(T,q0=joint_start)
                if sol.success:
                    goal_configuration = sol.q  # Joint configuration
                    print("Feasible joint configuration found")
                    break
            if not sol.success:
                for i in range(5):
                    sol = robot.ikine_LM(T)
                    if sol.success:
                        goal_configuration = sol.q  # Joint configuration
                        print("Feasible joint configuration found")
                        break

        # Check if the solution is valid
        if goal_configuration is not None:
             
            joint_distance = np.abs(np.subtract(joint_start, goal_configuration))
            max_joint_distance = np.max(joint_distance)
            step_num_joint = math.ceil(max_joint_distance / interp_dist_joint)
            # step_num_joint = int(np.ceil(np.linalg.norm(goal_configuration - joint_start) / interp_dist_joint))
            step_num=np.max([step_num_joint,step_num_lin,step_num_ori])+1

            pos_goal = np.vstack([np.linspace(start, end, step_num) for start, end in zip(position_start, [goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z])]).T
            joint_goal = np.vstack([np.linspace(start, end, step_num) for start, end in zip(joint_start, goal_configuration)]).T
            quat_goal = build_quat_seq(q_start_wxyz, q_goal_wxyz, step_num)

            
            i=0
            while i < step_num:
                qw, qx, qy, qz = quat_goal[i]
                pose_goal = pos_quat_2_pose_st(pos_goal[i], quaternion.quaternion(qw, qx, qy, qz))
                self.move_to_pose_with_stampedpose(pose_goal)
                self.set_configuration(joint_goal[i])
                if self.safety_checker():
                    i= i+1

                # r.sleep()
                # Per-step dwell paces the whole motion: larger = slower & gentler,
                # which avoids reflex errors on bigger pose changes.
                self.motion_sleep(0.01)
            
        else:
            self.fail_motion("No feasible joint configuration found")

    def go_to_pose_ik(self, goal_pose: PoseStamped, goal_configuration=None,
                    interp_dist=0.002, interp_dist_joint=0.008,
                    dt = 0.02,
                    ):
        self.set_stiffness(1000,1000,1000,80,80,80,0)
        # self.move_to_pose_with_stampedpose(self.curr_pose)
        # self.set_configuration(self.curr_joint)
        
        robot = rtb.models.Panda()

        pos_start = np.array(self.curr_pos, dtype=float)
        q_start_wxyz = q_norm([self.curr_pose.pose.orientation.w,
                            self.curr_pose.pose.orientation.x,
                            self.curr_pose.pose.orientation.y,
                            self.curr_pose.pose.orientation.z])

        goal_xyz = np.array([goal_pose.pose.position.x,
                            goal_pose.pose.position.y,
                            goal_pose.pose.position.z], dtype=float)
        q_goal_wxyz = q_norm([goal_pose.pose.orientation.w,
                            goal_pose.pose.orientation.x,
                            goal_pose.pose.orientation.y,
                            goal_pose.pose.orientation.z])

        # IK once
        if goal_configuration is None:
            Rg = q2r(q_goal_wxyz)
            T = SE3.Rt(Rg, goal_xyz)
            sol = robot.ikine_LM(T, q0=np.array(self.curr_joint))
            if not sol.success:
                sol = robot.ikine_LM(T)
            if not sol.success:
                self.fail_motion("No feasible joint configuration found")
            goal_configuration = np.asarray(sol.q, dtype=float)
        else:
            goal_configuration = np.asarray(goal_configuration, dtype=float)

        joint_start = np.asarray(self.curr_joint, dtype=float)

        # ---------- coarse/adaptive step counts ----------
        # Position & joints (coarse for speed)
        lin_dist = float(np.linalg.norm(goal_xyz - pos_start))
        step_lin = max(1, int(math.ceil(lin_dist / interp_dist)))
        max_joint = float(np.max(np.abs(goal_configuration - joint_start)))
        step_jnt = max(1, int(math.ceil(max_joint / interp_dist_joint)))       # ~0.08 rad

        # Orientation (allow large steps)
        max_ori_step = HIGH_ORI_DIFFERENCE / 4  # leave room for tracking lag
        ori_dist = q_angle(q_start_wxyz, q_goal_wxyz)
        step_ori = max(1, int(math.ceil(ori_dist / max_ori_step)))

        step_num = int(max(step_lin, step_jnt, step_ori)) + 1

        # Build sequences
        pos_seq   = np.vstack([np.linspace(s, g, step_num) for s, g in zip(pos_start, goal_xyz)]).T
        quat_seq  = build_quat_seq(q_start_wxyz, q_goal_wxyz, step_num)

        i = 0
        while i < step_num:
            qw, qx, qy, qz = quat_seq[i]
            pose_goal = pos_quat_2_pose_st(pos_seq[i], quaternion.quaternion(qw, qx, qy, qz))
            self.move_to_pose_with_stampedpose(pose_goal)

            self.motion_sleep(dt)
            if self.safety_checker():
                i += 1

        # ---------- brief, capped orientation refinement (<= 0.3s) ----------
        # Only if needed; bigger step for speed, small cap on duration.
        def refine_quat(max_time_s=0.30):
            start_t = time.monotonic()
            ang_tol  = math.radians(0.6)   # ~0.6°
            max_step = HIGH_ORI_DIFFERENCE / 4
            gamma    = 0.6                 # aggressive correction
            while (time.monotonic() - start_t) < max_time_s:
                q_curr = q_norm([self.curr_pose.pose.orientation.w,
                                self.curr_pose.pose.orientation.x,
                                self.curr_pose.pose.orientation.y,
                                self.curr_pose.pose.orientation.z])
                ang_err = q_angle(q_curr, q_goal_wxyz)
                if ang_err <= ang_tol:
                    break
                frac = min(gamma, max_step / max(ang_err, 1e-6))
                q_next = q_slerp(q_curr, q_goal_wxyz, frac)
                self.move_to_pose_with_stampedpose(pos_quat_2_pose_st(goal_xyz, quaternion.quaternion(*q_next)))
                self.motion_sleep(0.006)

        if ori_dist > math.radians(0.3):  # skip if orientation change was tiny
            refine_quat(max_time_s=0.30)

        # Final exact goal (cheap) and short settle
        self.move_to_pose_with_stampedpose(goal_pose)
        self.motion_sleep(0.15)

    def safety_checker(self):
        distance = np.linalg.norm(self.curr_pos_goal - self.curr_pos)
        angle = q_angle(q_norm(self.curr_ori_goal_wxyz), q_norm(self.curr_ori_wxyz))
        self.safety_check = self.check_tracking(
            distance <= self.attractor_distance_threshold and angle <= HIGH_ORI_DIFFERENCE / 2
        )
        return self.safety_check

    def offset_compensator(self, steps):
        curr_quat_desired= list_2_quaternion(np.copy(self.curr_ori_goal_wxyz))
        curr_pos_desired = np.copy(self.curr_pos_goal )
        for _ in range(steps):
            curr_quat_goal= list_2_quaternion(self.curr_ori_goal_wxyz)
            curr_pos_goal = self.curr_pos_goal 
            curr_quat = list_2_quaternion(self.curr_ori_wxyz)    
            
                    
            quat_diff = curr_quat_desired * curr_quat.inverse() 
            lin_diff = curr_pos_desired - self.curr_pos 
            
            
            quat_goal_new = quat_diff * curr_quat_goal
            goal_pos = curr_pos_goal + lin_diff
            
            goal_pose = pos_quat_2_pose_st(goal_pos, quat_goal_new)
            self.move_to_pose_with_stampedpose(goal_pose) 
            self.motion_sleep(0.2)
            

    def broadcast_transform(self):
        # Fetch robot state or hardcoded transformation for testing
        try:
            position = self.panda.get_position()  # XYZ position
            orientation = self.panda.get_orientation(scalar_first=False)  # Quaternion (x, y, z, w)
        except Exception as e:
            self.get_logger().error(f"Failed to get robot state: {e}")
            return

        # Create and populate TransformStamped message
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'panda_link0'
        transform_stamped.child_frame_id = 'panda_hand'

        # Assign translation
        transform_stamped.transform.translation.x = position[0]
        transform_stamped.transform.translation.y = position[1]
        transform_stamped.transform.translation.z = position[2]

        # Assign rotation
        transform_stamped.transform.rotation.x = orientation[0]
        transform_stamped.transform.rotation.y = orientation[1]
        transform_stamped.transform.rotation.z = orientation[2]
        transform_stamped.transform.rotation.w = orientation[3]

        # Publish the transform
        self.tf_broadcaster.sendTransform(transform_stamped)
        # self.get_logger().info(f"Published transform from 'panda_link0' to 'panda_hand'")

    def restart_control(self):
        self.check_motion()
        self.break_control_done.clear()
        self.break_control_requested.set()
        deadline = time.monotonic() + STOP_TIMEOUT
        while not self.break_control_done.wait(0.01):
            self.check_motion()
            if time.monotonic() >= deadline:
                self.fail_motion("Controller restart timed out")
        self.check_motion()

    def _control_step(self, ctrl):
        # Publishing and stop share this lock: no late waypoint can undo a stop.
        with self._motion_lock:
            if self._motion_cancel.is_set():
                if not self._hold_done.is_set():
                    ctrl.set_control(self.curr_pos, self.curr_ori_xyzw)
                    ctrl.set_impedance(np.diag([self.K_pos] * 3 + [self.K_ori] * 3))
                    self._hold_done.set()
                return
            if self.goal_position is None or self.goal_orientation is None:
                return
            try:
                self._validate_target(self.goal_position, self.goal_orientation)
            except MotionError:
                self._control_step(ctrl)  # service the newly latched hold
                return
            ctrl.set_control(self.goal_position, self.goal_orientation)

    def ctrl_node(self, frequency=500):
        while rclpy.ok():
            try:
                ctrl = controllers.CartesianImpedance(
                    filter_coeff=0.05,
                    impedance=np.diag([self.translational_stiffness_X, self.translational_stiffness_Y,
                                       self.translational_stiffness_Z, self.rotational_stiffness_X,
                                       self.rotational_stiffness_Y, self.rotational_stiffness_Z]),
                    nullspace_stiffness=self.nullspace_stiffness, damping_ratio=0.3)
                self.panda.start_controller(ctrl)
                with self.panda.create_context(frequency=frequency, max_runtime=999) as ctx:
                    # A restarted controller must acknowledge the hold itself.
                    self._hold_done.clear()
                    self._control_step(ctrl)
                    self._controller_ready.set()
                    self.break_control_done.set()
                    while ctx.ok() and rclpy.ok():
                        if self.break_control_requested.is_set():
                            self.break_control_requested.clear()
                            break
                        self._control_step(ctrl)
                        time.sleep(0.001)
            except Exception as exc:
                with self._motion_lock:
                    self._motion_fault = f"Controller failed: {exc}"
                    self.stop()
                self.get_logger().error(self._motion_fault)
                time.sleep(0.1)
            finally:
                self._controller_ready.clear()
                try:
                    self.panda.stop_controller()
                except Exception as exc:
                    self._motion_fault = f"Could not stop controller: {exc}"
                    self.stop()
                    self.get_logger().error(self._motion_fault)
                    return

    def _validate_target(self, position, orientation):
        position = np.asarray(position, dtype=float)
        orientation = np.asarray(orientation, dtype=float)
        if (position.shape != (3,) or orientation.shape != (4,)
                or not np.all(np.isfinite(position)) or not np.all(np.isfinite(orientation))
                or np.linalg.norm(orientation) == 0):
            self.fail_motion("Invalid target pose")
        distance = float(np.linalg.norm(position - self.curr_pos))
        try:
            angle = min_angle_condition(orientation, self.curr_ori_xyzw)
        except ValueError as exc:
            self.fail_motion(str(exc))
        if not np.isfinite(distance) or not np.isfinite(angle):
            self.fail_motion("Invalid robot pose")
        if distance > HIGH_POINT_DIFFERENCE or angle > HIGH_ORI_DIFFERENCE:
            self.fail_motion(f"Tracking limit exceeded: position={distance:.4f} m, orientation={angle:.4f} rad")

    def move_to_pose(self, position: Iterable[float], orientation: Iterable[float], speed_factor: float):
        with self._motion_lock:
            self.check_motion()
            self._validate_target(position, orientation)
            self.goal_position = tuple(position)
            self.goal_orientation = tuple(np.asarray(orientation) / np.linalg.norm(orientation))
            self.goal_q_nullspace = None

    def grasp(self, *args, **kwargs):
        self.check_motion()
        self.gripper.grasp(*args, **kwargs)

    def move(self, *args, **kwargs):
        self.check_motion()
        self.gripper.move(*args, **kwargs)

    @property
    def grip_value(self):
        return round(self.gripper_state.width, 2)

    @property
    def force(self): # Get current force 
        robot_state = self.panda.get_state()
        external_wrench = np.array(robot_state.O_F_ext_hat_K)  # [Fx, Fy, Fz, Tx, Ty, Tz]
        # Extract forces and torques
        return Point(x=external_wrench[0], y=external_wrench[1], z=external_wrench[2])  # [Fx, Fy, Fz]
        torques = external_wrench[3:]  # [Tx, Ty, Tz]

    @property
    def curr_pose(self): # Get current pose
        x0 = self.panda.get_position()
        q0 = self.panda.get_orientation(scalar_first=False)
        stamped_pose = PoseStamped(pose=Pose(
            position=Point(x=x0[0],y=x0[1],z=x0[2]),
            orientation=Quaternion(x=q0[0],y=q0[1],z=q0[2],w=q0[3])
        ))
        return stamped_pose

    @property
    def curr_pos(self):
        return self.panda.get_position()
    
    @property
    def curr_ori_xyzw(self):
        return self.panda.get_orientation(scalar_first=False)

    @property
    def curr_ori_wxyz(self):
        return self.panda.get_orientation(scalar_first=True)

    @property
    def curr_joint(self):
        return self.panda.get_state().q

    def update_params_thread(self):
        while rclpy.ok():
            time.sleep(UPDATE_THREAD_INTERVAL)
            
            last_stiffness = self.translational_stiffness_X, self.translational_stiffness_Y, self.translational_stiffness_Z, self.rotational_stiffness_X,self.rotational_stiffness_Y, self.rotational_stiffness_Z, self.nullspace_stiffness
            stiffness = get_remote_parameters(self, param_names=[
                "translational_stiffness_X",
                "translational_stiffness_Y",
                "translational_stiffness_Z",
                "rotational_stiffness_X",
                "rotational_stiffness_Y",
                "rotational_stiffness_Z",
                "nullspace_stiffness"
            ], server=self.get_name())

            if list(last_stiffness) != stiffness: # stiffness values changed!
                self.break_control_requested.set()

            self.translational_stiffness_X, self.translational_stiffness_Y, self.translational_stiffness_Z, self.rotational_stiffness_X,self.rotational_stiffness_Y, self.rotational_stiffness_Z, self.nullspace_stiffness = stiffness

    def broadcast_transform_thread(self):
        while rclpy.ok():
            time.sleep(TF_BROADCAST_INTERVAL)
            self.broadcast_transform()

    def feedback_thread(self):
        while rclpy.ok():
            pos = self.curr_pos
            ori = self.curr_ori_xyzw
            self.curr_pose_pub.publish(PoseStamped(pose=Pose(position=Point(x=pos[0], y=pos[1], z=pos[2]), orientation=Quaternion(x=ori[0], y=ori[1], z=ori[2], w=ori[3]))))
            self.publish_joint_state()
            time.sleep(0.1)

    def publish_joint_state(self):
        try:
            finger = self.grip_value / 2
            message = JointState(name=JOINT_NAMES,
                                 position=[*self.curr_joint, finger, finger])
        except Exception as error:
            self.get_logger().warning(f"Could not read joint state: {error}")
            return
        message.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_pub.publish(message)

    def gripper_state_thread(self):
        while rclpy.ok():
            time.sleep(0.5)
            self.gripper_state = self.gripper.read_once()

    def start(self):
        ctrl_thread = threading.Thread(target=self.ctrl_node, daemon=True)
        ctrl_thread.start()
        if not self._controller_ready.wait(10.0):
            raise MotionError("Controller did not start")
        if not DIRECT_STIFFNESS_OPTION:
            updateparam_thread = threading.Thread(target=self.update_params_thread, daemon=True)
            updateparam_thread.start()
        broadcast_transform_thread = threading.Thread(target=self.broadcast_transform_thread, daemon=True)
        broadcast_transform_thread.start()
        feedback_thread = threading.Thread(target=self.feedback_thread, daemon=True)
        feedback_thread.start()
        self.gripper_state = self.gripper.read_once() # Initialize gripper state
        gripper_read_thread = threading.Thread(target=self.gripper_state_thread, daemon=True)
        gripper_read_thread.start()


class SpinPandaNode(Panda, SpinningRosNode):
    def __init__(self):
        super(SpinPandaNode, self).__init__()

def main():
    import rclpy
    rclpy.init()
    panda = SpinPandaNode()
    panda.start()

    i = 0
    panda.goal_position = (0.4,0.0,0.4)
    panda.goal_orientation = (1.0,0.0,0.0,0.0)
    time.sleep(2.0)
    panda.set_stiffness(0,0,0,panda.K_ori,panda.K_ori,panda.K_ori,0)
    time.sleep(2.0)
    print("stiffnesses", panda.translational_stiffness_X, panda.translational_stiffness_Y, panda.translational_stiffness_Z, panda.rotational_stiffness_X, panda.rotational_stiffness_Y, panda.rotational_stiffness_Z, panda.nullspace_stiffness, flush=True)
    input("??")
    panda.set_stiffness(panda.K_pos,panda.K_pos,panda.K_pos,panda.K_ori,panda.K_ori,panda.K_ori,0)
    time.sleep(2.0)
    print("stiffnesses", panda.translational_stiffness_X, panda.translational_stiffness_Y, panda.translational_stiffness_Z, panda.rotational_stiffness_X, panda.rotational_stiffness_Y, panda.rotational_stiffness_Z, panda.nullspace_stiffness, flush=True)
    input("???")
    while rclpy.ok():
        i+=1
        panda.goal_position = (0.4+0.1*np.sin(i*0.01),0.0,0.4-0.1*np.cos(i*0.01))
        time.sleep(0.02)

if __name__ == "__main__":
    main()
