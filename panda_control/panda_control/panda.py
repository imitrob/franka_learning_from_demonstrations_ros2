#%%
#!/usr/bin/env python
import time, math
import quaternion
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import rclpy
import threading
from geometry_msgs.msg import PoseStamped
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
OPEN_GRIPPER_WIDTH = 0.06 # How much gripper opens [m]
HIGH_POINT_DIFFERENCE = 0.1 # m
HIGH_ORI_DIFFERENCE = 0.01

from typing import Iterable
# panda-py is chatty, activate information log level
import logging
logging.basicConfig(level=logging.WARNING)
from copy import deepcopy

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

class Panda():
    def __init__(self,
                 K_pos: int = 1000, # Default Positional stiffness
                 K_ori: int = 30, # Default Orientation stiffness
                 K_ns: int = 0, # Default Nullspace stiffness
                 ):
        super(Panda, self).__init__()
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

        self.tf_broadcaster = TransformBroadcaster(self)
        time.sleep(1)

        self.external_call_msg = None

        self.go_home_flag = False

    def has_realtime_kernel(self):
        return panda_py.libfranka.has_realtime_kernel()

    def is_grasped(self) -> bool:
        return self.gripper_state.is_grasped

    def IS_OPEN(self, value: float):
        return float(value) > self.grip_open_width / 2.0

    def is_open(self):
        return not self.gripper_state.is_grasped

    def external_call_handler(self):
        # if receives a target pose from topic, it goes there by linear motion
        while rclpy.ok():
            time.sleep(0.1)
            if self.external_call_msg is not None:
                pose = deepcopy(self.external_call_msg)
                self.external_call_msg = None
                self.go_to_pose_ik_quick(pose) # PoseStamped

    def external_call(self, msg):
        self.external_call_msg = msg
        # self.move_to_pose_with_stampedpose(msg) # old without linear motion

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
        self.gripper.stop()
        self.gripper.grasp(width=width, speed=0.05, force=50, epsilon_inner=0.055, epsilon_outer=0.055)

    def home(self, height=0.4, front_offset=0.4, side_offset=0.0):
        # go to joint target joints of home position
        self.restart_control(do_homing=True)
        # redundant:
        # go to position [0.4,0.0,0.4]
        self.move_to_pose_with_stampedpose(self.curr_pose)
        self.set_stiffness(self.K_pos, self.K_pos, self.K_pos, self.K_ori, self.K_ori, self.K_ori, 0)

        pos_array = np.array([front_offset, side_offset, height])
        quat = quaternion.quaternion(0, 1, 0, 0)
        goal = pos_quat_2_pose_st(pos_array, quat)
        goal.header.stamp = self.get_clock().now().to_msg()

        self.go_to_pose_ik(goal)

    def stop(self):
        self.goal_position = None

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
            r.sleep()
        self.move_to_pose_with_stampedpose(goal_pose)
        time.sleep(0.2)

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
        max_ori_step = math.radians(10.0)  # cap orientation change per step (~10 deg)
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
                if self.safety_check:
                    i= i+1

                # r.sleep()
                # Per-step dwell paces the whole motion: larger = slower & gentler,
                # which avoids reflex errors on bigger pose changes.
                time.sleep(0.01)

        else:
            print("No feasible joint configuration found or no joint configuration provided", flush=True)

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
                print("No feasible joint configuration found or no joint configuration provided", flush=True)
                return
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
        max_ori_step = math.radians(10.0)  # ~10 deg/step
        ori_dist = q_angle(q_start_wxyz, q_goal_wxyz)
        step_ori = max(1, int(math.ceil(ori_dist / max_ori_step)))

        step_num = int(min(max(step_lin, step_jnt, step_ori), 120)) + 1

        # Build sequences
        pos_seq   = np.vstack([np.linspace(s, g, step_num) for s, g in zip(pos_start, goal_xyz)]).T
        quat_seq  = build_quat_seq(q_start_wxyz, q_goal_wxyz, step_num)

        i = 0
        while i < step_num:
            qw, qx, qy, qz = quat_seq[i]
            pose_goal = pos_quat_2_pose_st(pos_seq[i], quaternion.quaternion(qw, qx, qy, qz))
            self.move_to_pose_with_stampedpose(pose_goal)

            time.sleep(dt)
            if self.safety_check:
                i += 1

        # ---------- brief, capped orientation refinement (<= 0.3s) ----------
        # Only if needed; bigger step for speed, small cap on duration.
        def refine_quat(max_time_s=0.30):
            start_t = time.time()
            ang_tol  = math.radians(0.6)   # ~0.6°
            max_step = math.radians(3.0)   # up to 3° per correction
            gamma    = 0.6                 # aggressive correction
            while (time.time() - start_t) < max_time_s:
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
                time.sleep(0.006)

        if ori_dist > math.radians(0.3):  # skip if orientation change was tiny
            refine_quat(max_time_s=0.30)

        # Final exact goal (cheap) and short settle
        self.move_to_pose_with_stampedpose(goal_pose)
        time.sleep(0.15)

    def safety_checker(self):
        distance_pos = np.linalg.norm(self.curr_pos_goal - self.curr_pos)
        if distance_pos < self.attractor_distance_threshold:
            self.safety_check = True
        else:
            self.get_logger().warning(f"Safety has been violated with distance {distance_pos}")
            self.safety_check = False


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
            time.sleep(0.2)


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

    def restart_control(self, do_homing = False):
        if do_homing:
            self.go_home_flag = True
            timeout = 10 # s (timeout with homeing)
        else:
            timeout = 2 # s (timeout just restart)

        self.break_control_done.clear()
        self.break_control_requested.set()
        if not self.break_control_done.wait(timeout=timeout):
            raise Exception("Restart request not finished in time!")

    def ctrl_node(self, frequency=500):
        while True:
            if self.go_home_flag:
                # self.panda.move_to_start()
                self.panda.move_to_joint_position(waypoints=[[0.0, -0.5, 0.0, -2.38, 0.0, 1.89, 0.8]])
                self.go_home_flag = False
            ctrl = controllers.CartesianImpedance(filter_coeff=0.05, impedance=np.diag([self.translational_stiffness_X, self.translational_stiffness_Y, self.translational_stiffness_Z, self.rotational_stiffness_X, self.rotational_stiffness_Y, self.rotational_stiffness_Z]), nullspace_stiffness=self.nullspace_stiffness, damping_ratio=0.3)
            # print("New ctrl:", self.translational_stiffness_X, self.translational_stiffness_Y, self.translational_stiffness_Z, self.rotational_stiffness_X, self.rotational_stiffness_Y, self.rotational_stiffness_Z)
            self.panda.start_controller(ctrl)
            try:
                with self.panda.create_context(frequency=frequency, max_runtime=999) as ctx:
                    self.break_control_done.set()
                    while ctx.ok():
                        if (self.goal_position is not None) and (self.goal_orientation is not None) and (self.curr_ori_xyzw is not None):
                            if (np.linalg.norm(np.array(self.goal_position) - np.array(self.curr_pos)) > HIGH_POINT_DIFFERENCE) or \
                                min_angle_condition(self.goal_orientation, self.curr_ori_xyzw) > HIGH_ORI_DIFFERENCE:

                                self.get_logger().warning(f"contror high set point difference {np.linalg.norm(np.array(self.goal_position) - np.array(self.curr_pos))}  {min_angle_condition(self.goal_orientation, self.curr_ori_xyzw) > HIGH_ORI_DIFFERENCE}")

                                # direction = (np.array(self.goal_position) - np.array(self.curr_pos)) / np.linalg.norm(np.array(self.goal_position) - np.array(self.curr_pos))
                                # new_goal_position = self.curr_pos + direction * HIGH_POINT_DIFFERENCE * 0.5

                                # new_goal_orientation = step_slerp(
                                #     self.goal_orientation,
                                #     self.curr_ori_xyzw,
                                #     HIGH_ORI_DIFFERENCE * 0.5,
                                # )

                                # self.get_logger().warning(f"{self.curr_pos}, {self.curr_ori_xyzw}, || , {new_goal_position}, {new_goal_orientation}")

                                # ctrl.set_control(new_goal_position, new_goal_orientation)
                                time.sleep(0.001) # Needed! Enforce consistent rate on non rt PC
                                continue

                        if (self.goal_position is not None) and (self.goal_orientation is not None):
                            ctrl.set_control(self.goal_position, self.goal_orientation)
                        time.sleep(0.001) # Needed! Enforce consistent rate on non rt PC
                        # if self.break_control_requested:
                        if self.break_control_requested.wait(timeout = 0.001):
                            self.break_control_requested.clear()
                            # print("Restarting control")
                            self.panda.stop_controller()
                            break
            except RuntimeError as e:
                print(f"Recovering from libfranka exception: {str(e)}", flush=True)

    def move_to_pose(self,
                     position: Iterable[float], # xyz
                     orientation: Iterable[float], # xyzw
                     speed_factor: float,
                    ):
        self.goal_position = tuple(position)
        self.goal_orientation = tuple(orientation)
        self.goal_q_nullspace = None

    def grasp(self, *args, **kwargs):
        self.gripper.grasp(*args, **kwargs)

    def move(self, *args, **kwargs):
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
            time.sleep(UPDATE_THREAD_INTERVAL)
            self.broadcast_transform()

    def feedback_thread(self):
        while rclpy.ok():
            pos = self.curr_pos
            ori = self.curr_ori_xyzw
            self.curr_pose_pub.publish(PoseStamped(pose=Pose(position=Point(x=pos[0], y=pos[1], z=pos[2]), orientation=Quaternion(x=ori[0], y=ori[1], z=ori[2], w=ori[3]))))
            time.sleep(0.1)

    def gripper_state_thread(self):
        while rclpy.ok():
            time.sleep(0.5)
            self.gripper_state = self.gripper.read_once()

    def start(self):
        ctrl_thread = threading.Thread(target=self.ctrl_node, daemon=True)
        ctrl_thread.start()
        if not DIRECT_STIFFNESS_OPTION:
            updateparam_thread = threading.Thread(target=self.update_params_thread, daemon=True)
            updateparam_thread.start()
        broadcast_transform_thread = threading.Thread(target=self.broadcast_transform_thread, daemon=True)
        broadcast_transform_thread.start()
        feedback_thread = threading.Thread(target=self.feedback_thread, daemon=True)
        feedback_thread.start()
        external_call_handler = threading.Thread(target=self.external_call_handler, daemon=True)
        external_call_handler.start()
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

