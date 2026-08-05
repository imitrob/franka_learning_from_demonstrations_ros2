from __future__ import annotations
import rclpy
import numpy as np
from pynput.keyboard import KeyCode, Key
from pynput.keyboard import Listener
from panda_control.pose_transform_functions import pos_quat_2_pose_st, list_2_quaternion
from std_msgs.msg import Float32, Bool
import time
from dataclasses import dataclass

from nocode_robot_programming.joystick import JoystickConnector
from nocode_robot_programming.gestures import TeleoperationByDrawing

import threading

BUTTON_PRESS_MODE = "momentary"

class KeyboardConnector():
    def __init__(self):
        super(KeyboardConnector, self).__init__()
        self.key_thr_running = False
    
    def keyboard_start(self):
        if not self.key_thr_running:
            self.key_thr_running = True
            self.key_thr = threading.Thread(target=self.keyboard_start_thread, daemon=True)
            self.key_thr.start()

    def keyboard_start_thread(self):
        self.keyboard_listener = Listener(on_press=self.keyboard_on_press, on_release=self.keyboard_on_release)
        self.keyboard_listener.start()
        self.keyboard_listener.join()  # keep the program alive (optional in REPL, essential in scripts)

    def keyboard_stop(self):
        if self.key_thr_running:
            self.key_thr.join(timeout=1)
            self.keyboard_listener.stop()
            self.key_thr_running = False
    

KEY_MAP = {
    'check': 'check',
    'cross': 'cross',
    'circle': 'circle',
    'left': 'x_negative',
    'right': 'x_positive',
    'down': 'y_negative',
    'up': 'y_positive',
}

class FrankaOnPress():
    def __init__(self):
        super(FrankaOnPress, self).__init__()

        self.x_positive_press_act = False
        self.x_negative_press_act = False
        self.y_positive_press_act = False
        self.y_negative_press_act = False
        self.circle_press_act = False
        self.cross_press_act = False
        self.check_press_act = False

        self.x_positive_release_act = False
        self.x_negative_release_act = False
        self.y_positive_release_act = False
        self.y_negative_release_act = False
        self.circle_release_act = False
        self.cross_release_act = False
        self.check_release_act = False

        self.frankabuttons_running = False
        
        # self.frankabuttons_start() # Franka buttons must be manually initialized now

    def frankabuttons_start(self):
        if not self.frankabuttons_running:
            self.frankabuttons_running = True
            self.desk.listen(self.franka_button_callback)
        '''
        {'check': False, 'circle': False, 'cross': False, 'down': False, 'left': False, 'right': False, 'up': False}
        '''

    def frankabuttons_stop(self):
        # panda_py's Desk.listen() thread is NOT a daemon, so the process hangs
        # on exit unless it is joined explicitly.
        if self.frankabuttons_running:
            self.desk.stop_listen()
            self.frankabuttons_running = False

    def franka_button_callback(self, event_dict):
        for key in event_dict:
            self.decide_act(KEY_MAP[key], event_dict[key])

    def cb_x(self, msg):
        if int(msg.data) == 1:
            self.decide_act('x_positive', True)
        elif int(msg.data) == -1:
            self.decide_act('x_negative', True)
        elif int(msg.data) == 0:
            self.decide_act('x_positive', False)
            self.decide_act('x_negative', False)
        else: raise Exception()

    def cb_y(self, msg):
        if int(msg.data) == 1:
            self.decide_act('y_positive', True)
        elif int(msg.data) == -1:
            self.decide_act('y_negative', True)
        elif int(msg.data) == 0:
            self.decide_act('y_positive', False)
            self.decide_act('y_negative', False)
        else: raise Exception()

    def cb_circle(self, msg):
        self.decide_act('circle', bool(msg.data))

    def cb_cross(self, msg):
        self.decide_act('cross', bool(msg.data))

    def cb_check(self, msg):    
        self.decide_act('check', bool(msg.data))

    def decide_act(self, btn: str, clicked: bool):
        if clicked:
            act_already = getattr(self, btn+'_press_act')
            if not act_already:
                setattr(self, btn+'_press_act', True)
                self.franka_on_press(btn)        
            setattr(self, btn+'_release_act', False)    
        else:
            act_already = getattr(self, btn+'_release_act')
            if not act_already:
                setattr(self, btn+'_release_act', True)
                self.franka_on_release(btn)    
            setattr(self, btn+'_press_act', False)

    def franka_on_press(self, btn):
        if btn == "check":
            print("Event happened, user pressed Check")
        elif btn == "cross":
            print("Event happened, user pressed Cross")
        elif btn == "circle":
            print("Event happened, user pressed Circle")
        elif btn == "x_positive":
            print("Event happened, user pressed x=1")
        elif btn == "x_negative":
            print("Event happened, user pressed x=-1")
        elif btn == "y_positive":
            print("Event happened, user pressed y=1")
        elif btn == "y_negative":
            print("Event happened, user pressed y=-1")

    def franka_on_release(self, btn):
        if btn == "check":
            print("Event happened, user released Check")
        elif btn == "cross":
            print("Event happened, user released Cross")
        elif btn == "circle":
            print("Event happened, user released Circle")
        elif btn == "x_positive":
            print("Event happened, user released x=1")
        elif btn == "x_negative":
            print("Event happened, user released x=-1")
        elif btn == "y_positive":
            print("Event happened, user released y=1")
        elif btn == "y_negative":
            print("Event happened, user released y=-1")



class FrankaConnector(FrankaOnPress):
    def __init__(self):
        super(FrankaConnector, self).__init__()
    def franka_on_press(self, key):
        if key == "check":
            self.keyboard_on_press(KeyCode.from_char("o"))
            # self.keyboard_on_press(KeyCode.from_char("t")) # safe
        elif key == "cross":
            self.keyboard_on_press(KeyCode.from_char("c"))
            # self.keyboard_on_press(KeyCode.from_char("r")) # danger
        elif key == "circle":
            # self.keyboard_on_press(KeyCode.from_char("q"))
            self.keyboard_on_press(KeyCode.from_char("e"))
        elif key == "x_positive":
            # print("x=1 button have no mapping")
            self.keyboard_on_press(KeyCode.from_char("c"))
        elif key == "x_negative":
            # print("x=-1 button have no mapping")
            self.keyboard_on_press(KeyCode.from_char("o"))
        elif key == "y_positive":
            print("y=1 button have no mapping")
        elif key == "y_negative":
            print("y=-1 button have no mapping")

    def franka_on_release(self, key):
        if key == "check":
            print("transparent (safe) flag enabled")
            self.safe_flag = 1
            self.risk_flag = 0
        elif key == "cross":
            self.keyboard_on_release(KeyCode.from_char("r")) # danger
        elif key == "circle":
            self.keyboard_on_release(KeyCode.from_char("q"))
        elif key == "x_positive":
            print("x=1 button have no mapping")
        elif key == "x_negative":
            print("x=-1 button have no mapping")
        elif key == "y_positive":
            print("y=1 button have no mapping")
        elif key == "y_negative":
            print("y=-1 button have no mapping")




class Feedback(FrankaConnector, KeyboardConnector, JoystickConnector, TeleoperationByDrawing):
    def __init__(self):
        super(Feedback, self).__init__()

        # Variables control the robot when demonstration recording
        self.joystick_feedback = None # [0.0,0.0,0.0] # relative feedback active gains
        self.gesture_feedback = None # [0.0,0.0,0.0] # absolute feedback toward base

        self.rot_feedback = [0.0, 0.0]
        
        self.feedback_gripper = ""

        self.correction_feedback=np.zeros(4)
        self.feedback_gain=0.002
        self.faster_counter=0
        self.length_scale = 0.005
        self.correction_window = 300
        self.img_feedback_flag = 0
        self.spiral_flag = 0
        self.img_feedback_correction = 0
        # self.gripper_feedback_correction = 0
        self.spiral_feedback_correction=0
        self.pause=False

    def is_applied_external_feedback(self):
        if (self.joystick_feedback is not None) or (self.gesture_feedback is not None):
            return True
        else:
            return False

    def input_check(self):
        try:
            while True:
                time.sleep(0.2)
                if self.joystick_feedback is not None:
                    print(f"{self.joystick_feedback[0]:+.3f} {self.joystick_feedback[1]:+.3f} {self.joystick_feedback[2]:+.3f} {self.rot_feedback[0]:+.3f} {self.rot_feedback[1]:+.3f}. {str(self.feedback_gripper):.7s}, pause: {str(self.pause):.5s}", end="\r", flush=True)
                if self.gesture_feedback is not None:
                    print(f"{self.gesture_feedback[0]:+.3f} {self.gesture_feedback[1]:+.3f} {self.gesture_feedback[2]:+.3f} {self.rot_feedback[0]:+.3f} {self.rot_feedback[1]:+.3f}. {str(self.feedback_gripper):.7s}, pause: {str(self.pause):.5s}", end="\r", flush=True)
        except KeyboardInterrupt:
            return

    def keyboard_on_release(self, key):
        pass

    ## I must find better solution how to get the robot handle
    @property
    def _robot(self):
        if self.__class__.__name__ == "LfD" or self.__class__.__name__ == "RALfD":
            return self
        elif hasattr(self, "LfD") and self.LfD is not None:
            return self.LfD
        elif hasattr(self, "LfD") and self.LfD is None:
            self.init_lfd()
            return self.LfD
        else:
            print(f"No robot available", flush=True)

    def keyboard_on_press(self, key):
        # self.get_logger().info(f"Event happened, user pressed {key}")
        # This function runs on the background and checks if a keyboard key was pressed
        if key == KeyCode.from_char('e'):
            self.end += 1
        # Feedback for translate forward/backward
        if key == KeyCode.from_char('w'):
            self.correction_feedback[0] = self.feedback_gain
        if key == KeyCode.from_char('s'):
            self.correction_feedback[0] = -self.feedback_gain
        # Feedback for translate left/right
        if key == KeyCode.from_char('a'):
            self.correction_feedback[1] = self.feedback_gain
        if key == KeyCode.from_char('d'):
            self.correction_feedback[1] = -self.feedback_gain
        # Feedback for translate up/down
        if key == KeyCode.from_char('u'):
            self.correction_feedback[2] = self.feedback_gain
        if key == KeyCode.from_char('j'):
            self.correction_feedback[2] = -self.feedback_gain
        # Close/open gripper
        if key == KeyCode.from_char('c'):
            if self._robot is not None:
                if not self._robot.gripper_state.is_grasped:
                    self._robot.grasp_gripper(0)
                # self.gripper_feedback_correction = 1

        if key == KeyCode.from_char('o'):
            if self._robot is not None:
                self._robot.move_gripper(self._robot.grip_open_width)
                # self.gripper_feedback_correction = 1
        if key == KeyCode.from_char('f'):
            self.correction_feedback[3] = 1
        if key == KeyCode.from_char('k'):
            print("camera feedback enabled")
            self.img_feedback_flag = 1
            self.img_feedback_correction = 1
        if key == KeyCode.from_char('l'):
            print("camera feedback disabled")
            self.img_feedback_flag = 0
            self.img_feedback_correction = 1
        if key == KeyCode.from_char('z'):
            print("spiral enabled")
            self.spiral_flag = 1
            self.spiral_feedback_correction=1
        if key == KeyCode.from_char('x'):
            print("spiral disabled")
            self.spiral_feedback_correction=1
            self.spiral_flag = 0

        if key == KeyCode.from_char('m'):    
            if self._robot is not None:
                quat_goal = list_2_quaternion(self._robot.curr_ori_wxyz)
                goal = pos_quat_2_pose_st(self._robot.curr_pos, quat_goal)

                self._robot.move_to_pose_with_stampedpose(goal)
                
                self._robot.set_stiffness(0, 0, 0, 50, 50, 50, 0)
                print("higher rotatioal stiffness")

        if key == KeyCode.from_char('n'):
            if self._robot is not None:
                self._robot.set_stiffness(0, 0, 0, 0, 0, 0, 0)
                print("zero rotatioal stiffness")
        if key == Key.space:
            self.pause=not(self.pause)
            if self.pause==True:
                print("Recording paused")    
            else:
                print("Recording started again")  
        key=0

    def square_exp(self, ind_curr, ind_j):
        dist = np.sqrt((self.loaded_traj[0][ind_curr]-self.loaded_traj[0][ind_j])**2+(self.loaded_traj[1][ind_curr]-self.loaded_traj[1][ind_j])**2+(self.loaded_traj[2][ind_curr]-self.loaded_traj[2][ind_j])**2)
        sq_exp = np.exp(-dist**2/self.length_scale**2)
        return sq_exp    

    def correct(self):
        if np.sum(self.correction_feedback[:3])!=0:
            for j in range(self.loaded_traj.shape[1]):
                x = self.correction_feedback[0]*self.square_exp(self.time_index, j)
                y = self.correction_feedback[1]*self.square_exp(self.time_index, j)
                z = self.correction_feedback[2]*self.square_exp(self.time_index, j)

                self.loaded_traj[0][j] += x
                self.loaded_traj[1][j] += y
                self.loaded_traj[2][j] += z
        
        if self.img_feedback_correction:
            self.loaded_img_feedback_flag[0, self.time_index:] = self.img_feedback_flag

        if self.spiral_feedback_correction:
            self.loaded_spiral_flag[0, self.time_index:] = self.spiral_flag
        
        # if self.gripper_feedback_correction:
        #     self.loaded_gripper[0, self.time_index:] = self.grip_value

        if self.correction_feedback[3] != 0:
            self.faster_counter = 10
            
        if self.faster_counter > 0 and self.time_index != self.loaded_traj.shape[1]-1:
            self.faster_counter -= 1
            self.loaded_traj = np.delete(self.loaded_traj, self.time_index+1, 1)
            self.loaded_ori_wxyz = np.delete(self.loaded_ori_wxyz, self.time_index+1, 1)
            self.loaded_gripper = np.delete(self.loaded_gripper, self.time_index+1, 1)
            self.loaded_img = np.delete(self.loaded_img, self.time_index+1, 0)
            self.loaded_img_feedback_flag = np.delete(self.loaded_img_feedback_flag, self.time_index+1, 1)
            self.loaded_spiral_flag = np.delete(self.loaded_spiral_flag, self.time_index+1, 1)
                       
        self.correction_feedback = np.zeros(4)
        self.img_feedback_correction = 0
        # self.gripper_feedback_correction = 0
        self.spiral_feedback_correction = 0 

    


class RiskAwareFeedback(Feedback):
    def __init__(self, button_press_mode: str = BUTTON_PRESS_MODE):
        super(RiskAwareFeedback, self).__init__()
        self.risk_flag = 0
        self.safe_flag = 0
        self.novelty_flag = 0
        self.recovery_phase = -1.

        try:
            self.button_press_mode
        except AttributeError:
            self.button_press_mode = button_press_mode

    def keyboard_on_press(self, key):
        if not hasattr(self,'button_press_mode'): return # init not finished

        if self.button_press_mode == 'toggle':
            self._on_press_toggle(key)
        elif self.button_press_mode == 'momentary':
            self._on_press_momentary(key)
        else: raise Exception()

    def keyboard_on_release(self, key):
        if not hasattr(self,'button_press_mode'): return # init not finished

        if self.button_press_mode == 'toggle':
            pass
        elif self.button_press_mode == 'momentary':
            self._on_release_momentary(key)
        else: raise Exception()


    def _on_press_toggle(self, key):
        if key == KeyCode.from_char("r"):
            print("risk flag enabled")
            self.risk_flag = 1
            self.safe_flag = 0
        if key == KeyCode.from_char("t"):
            print("transparent (safe) flag enabled")
            self.safe_flag = 1
            self.risk_flag = 0
        if key == KeyCode.from_char("p"):
            print("phenomenon (novelty) flag enabled")
            self.novelty_flag = 1
        if key == KeyCode.from_char("q"):
            print("all risk-related flags disabled")
            self.risk_flag = 0
            self.safe_flag = 0
            self.novelty_flag = 0
            self.recovery_phase = -1.0

        if key == KeyCode.from_char("+"):
            self.target_time_index += 20
        if key == KeyCode.from_char("-"):
            self.target_time_index -= 20

        for i in range(10):
            if key == KeyCode.from_char(str(i)):
                fraction = float(i) / 10
                self.target_time_index = int(fraction * self.loaded_trajectory_len)
                self.recovery_phase = fraction

        super().keyboard_on_press(key)

    def _on_press_momentary(self, key):
        if key == KeyCode.from_char("r"):
            print("risk flag enabled")
            self.risk_flag = 1
        if key == KeyCode.from_char("t"):
            print("transparent (safe) flag enabled")
            self.safe_flag = 1
        if key == KeyCode.from_char("p"):
            print("phenomenon (novelty) flag enabled")
            self.novelty_flag = 1

        if key == KeyCode.from_char("+"):
            self.target_time_index += 20
        if key == KeyCode.from_char("-"):
            self.target_time_index -= 20

        for i in range(10):
            if key == KeyCode.from_char(str(i)):
                fraction = float(i) / 10
                self.target_time_index = int(fraction * self.loaded_trajectory_len)
                self.recovery_phase = fraction

        super().keyboard_on_press(key)

    def _on_release_momentary(self, key):
        if key == KeyCode.from_char("r"):
            self.risk_flag = 0
        if key == KeyCode.from_char("t"):
            self.safe_flag = 0
        if key == KeyCode.from_char("p"):
            self.novelty_flag = 0
        for i in range(10):
            if key == KeyCode.from_char(str(i)):
                self.recovery_phase = -1.0


if __name__ == '__main__':
    from skills_manager.ros_utils import SpinningRosNode

    @dataclass
    class DummyGripperState:
        is_grasped: bool

    @dataclass
    class DummyGripper:
        state: DummyGripperState
        def read_once(self):
            return self.state

    class DummyPanda:
        def get_position(self):
            return [0.4, 0.0, 0.4]
        def get_orientation(self, scalar_first: bool):
            return [1.0, 0.0, 0.0, 0.0]

    class FeedbackRosNode(Feedback, SpinningRosNode):
        def __init__(self):
            super(FeedbackRosNode, self).__init__()
            self.end = False
            self.gripper = DummyGripper(DummyGripperState(is_grasped=False))
            self.panda = DummyPanda()

    rclpy.init()
    # fb = FrankaButtons()
    
    f = FeedbackRosNode()
    f.keyboard_start()
    # f.keyboard_stop()
    # f.keyboard_start()
    f.joy_start()
    # f.joy_stop()
    # f.joy_start()
    f.teleop_start()

    # raf = RiskAwareFeedback(button_press_mode="toggle")
    # raf = RiskAwareFeedback(button_press_mode="momentary")
    while True:
        time.sleep(0.1)
        print(f.feedback, f.end, f.end > 2, flush=True)

