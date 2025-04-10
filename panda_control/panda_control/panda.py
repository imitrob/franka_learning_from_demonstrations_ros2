#%%
#!/usr/bin/env python
import time
import math
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import rclpy
import threading
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# from franka_gripper.msg import GraspActionGoal, HomingActionGoal, StopActionGoal, MoveActionGoal
from panda_control.pose_transform_functions import  pos_quat_2_pose_st, list_2_quaternion, pose_2_transformation, interpolate_poses
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
HOSTNAME = "192.168.89.140"
username = 'admin'
password = '123456789'

UPDATE_THREAD_INTERVAL = 1.0 # s

from typing import Iterable
# panda-py is chatty, activate information log level
import logging
logging.basicConfig(level=logging.INFO)
from copy import deepcopy

class Panda():
    def __init__(self,
                 K_pos: int = 600, # Default Positional stiffness
                 K_ori: int = 50, # Default Orientation stiffness
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
        self.safety_check=True
         
        self.gripper_width = 0

        self.translational_stiffness_X = self.K_pos
        self.translational_stiffness_Y = self.K_pos
        self.translational_stiffness_Z = self.K_pos
        self.rotational_stiffness_X = self.K_ori
        self.rotational_stiffness_Y = self.K_ori
        self.rotational_stiffness_Z = self.K_ori
        self.nullspace_stiffness = self.K_ns
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
        
        self.gripper = Gripper(HOSTNAME)
        self.goal_position = None # Set (x,y,z) attractor
        self.goal_orientation = None # Set (1.0,0.0,0.0,0.0) attractor ori xyzw https://jeanelsner.github.io/panda-py/panda_py.html#panda_py.Panda.move_to_pose
        self.goal_q_nullspace = None

        self.break_control_flag = False # e.g. if stiffness changed request
        
        self.create_subscription(PoseStamped, "/panda/goal_pose", self.external_call, 5)
        self.curr_pose_pub = self.create_publisher(PoseStamped, "/panda/curr_pose", 5)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        time.sleep(1)

        self.external_call_msg = None

    def external_call_handler(self): 
        # if receives a target pose from topic, it goes there by linear motion
        while rclpy.ok():
            time.sleep(0.1)
            if self.external_call_msg is not None:
                pose = deepcopy(self.external_call_msg)
                self.external_call_msg = None
                self.go_to_pose_ik(pose) # PoseStamped

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
        
    
    def move_gripper(self,width):
        self.move(width, speed=0.05)

    def grasp_gripper(self, width):
        self.gripper.grasp(width=width, speed=0.05, force=10, epsilon_inner=0.055, epsilon_outer=0.055)

    def home(self, height=0.4, front_offset=0.4, side_offset=0.0):
        self.move_to_pose_with_stampedpose(self.curr_pose)
        self.set_stiffness(self.K_pos, self.K_pos, self.K_pos, self.K_ori, self.K_ori, self.K_ori, 0)

        pos_array = np.array([front_offset, side_offset, height])
        quat = np.quaternion(0, 1, 0, 0)
        goal = pos_quat_2_pose_st(pos_array, quat)
        goal.header.stamp = self.get_clock().now().to_msg()

        # ns_msg = [0, 0, 0, -2.4, 0, 2.4, 0.8] #ensure that the elbow is upward
        self.go_to_pose_ik(goal)#, goal_configuration=ns_msg) # TODO: Works, but needs tuning

    def home_gripper(self):
        self.gripper.homing()
        # self.homing_pub.publish(self.home_command)

    def stop_gripper(self):
        self.gripper.stop()
        # self.stop_pub.publish(self.stop_command)  

    def joint_states_callback(self, data):
        self.curr_joint = data.position[:7]
        self.gripper_width = data.position[7] + data.position[8]
    
    def set_configuration(self,joint):
        joint_des=Float32MultiArray()
        joint_des.data= np.array(joint).astype(np.float32).tolist()
        # self.configuration_pub.publish(joint_des)
        self.goal_q_nullspace = tuple(joint)
    def set_stiffness(self, k_t1: int, k_t2: int, k_t3: int,k_r1: int,k_r2: int, k_r3: int, k_ns: int):
        
        set_remote_parameters(self, [
            "translational_stiffness_X", "translational_stiffness_Y", "translational_stiffness_Z",
            "rotational_stiffness_X", "rotational_stiffness_Y", "rotational_stiffness_Z", "nullspace_stiffness"
            ], [k_t1, k_t2, k_t3, k_r1, k_r2, k_r3, k_ns], server=self.get_name())

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
    def go_to_pose_ik(self, goal_pose: PoseStamped, goal_configuration=None, interp_dist=0.002, interp_dist_joint=0.004): 
        # the goal pose should be of type PoseStamped. E.g. goal_pose=PoseStampled()
        # set_parameter(self,self.get_name(),"max_delta_lin",0.2)
        # set_parameter(self,self.get_name(),"max_delta_ori",0.5)
        r = self.create_rate(200)
        self.move_to_pose_with_stampedpose(self.curr_pose)
        
        self.set_configuration(self.curr_joint)
        self.set_stiffness(self.K_pos, self.K_pos, self.K_pos, self.K_ori, self.K_ori, self.K_ori, self.K_ns) # Note: Legacy had zeros stiffness for rotations

        robot = rtb.models.Panda()
        position_start = self.curr_pos
        joint_start = np.array(self.curr_joint)
        goal_array = np.array([goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z])

        # interpolate from start to goal with attractor distance of approx 1 cm
        dist = np.sqrt(np.sum(np.subtract(position_start, goal_array)**2, axis=0))
        
        step_num_lin = math.floor(dist / interp_dist)
        q_goal=np.quaternion(goal_pose.pose.orientation.w, goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z)
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
            step_num=np.max([step_num_joint,step_num_lin])+1
        
            pos_goal = np.vstack([np.linspace(start, end, step_num) for start, end in zip(position_start, [goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z])]).T
            joint_goal = np.vstack([np.linspace(start, end, step_num) for start, end in zip(joint_start, goal_configuration)]).T

            # self.set_stiffness(self.K_pos, self.K_pos, self.K_pos, 0, 0, 0, 0)

            i=0
            while i < step_num:
                pose_goal = pos_quat_2_pose_st(pos_goal[i], q_goal) 
                self.move_to_pose_with_stampedpose(pose_goal)
                self.set_configuration(joint_goal[i])
                if self.safety_check:
                    i= i+1 

                # r.sleep()
                time.sleep(0.01)
            self.set_stiffness(self.K_pos, self.K_pos, self.K_pos, self.K_ori, self.K_ori, self.K_ori, 0)

            time.sleep(1) 
            self.move_to_pose_with_stampedpose(goal_pose)

        else:
            print("No feasible joint configuration found or no joint configuration provided", flush=True)
        
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

    def restart_control(self):
        self.break_control_flag = True

    def ctrl_node(self, frequency=500):
        while True:
            ctrl = controllers.CartesianImpedance(filter_coeff=0.5, impedance=np.diag([self.translational_stiffness_X, self.translational_stiffness_Y, self.translational_stiffness_Z, self.rotational_stiffness_X, self.rotational_stiffness_Y, self.rotational_stiffness_Z]), nullspace_stiffness=self.nullspace_stiffness, damping_ratio=0.5)
            self.panda.start_controller(ctrl)
            try:
                with self.panda.create_context(frequency=frequency, max_runtime=999) as ctx:
                    while ctx.ok():
                        if (self.goal_position is not None) and (self.goal_orientation is not None):
                            ctrl.set_control(self.goal_position, self.goal_orientation)
                        time.sleep(0.001) # Needed! Enforce consistent rate on non rt PC
                        if self.break_control_flag:
                            print("Restarting control!", flush=True)
                            self.panda.stop_controller()
                            self.break_control_flag = False
                            break
            except:
                pass

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
                self.break_control_flag = True

            self.translational_stiffness_X, self.translational_stiffness_Y, self.translational_stiffness_Z, self.rotational_stiffness_X,self.rotational_stiffness_Y, self.rotational_stiffness_Z, self.nullspace_stiffness = stiffness

            self.broadcast_transform()

    def feedback_thread(self):
        while rclpy.ok():
            time.sleep(0.1)
            pos = self.curr_pos
            ori = self.curr_ori_xyzw
            self.curr_pose_pub.publish(PoseStamped(pose=Pose(position=Point(x=pos[0], y=pos[1], z=pos[2]), orientation=Quaternion(x=ori[0], y=ori[1], z=ori[2], w=ori[3]))))

    def start(self):
        ctrl_thread = threading.Thread(target=self.ctrl_node, daemon=True)
        ctrl_thread.start()
        updateparam_thread = threading.Thread(target=self.update_params_thread, daemon=True)
        updateparam_thread.start()
        feedback_thread = threading.Thread(target=self.feedback_thread, daemon=True)
        feedback_thread.start()
        external_call_handler = threading.Thread(target=self.external_call_handler, daemon=True)
        external_call_handler.start()


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

    