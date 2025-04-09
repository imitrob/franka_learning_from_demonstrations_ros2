import rclpy
import numpy as np
from pynput.keyboard import KeyCode, Key
from pynput.keyboard import Listener
from panda_control.pose_transform_functions import pos_quat_2_pose_st, list_2_quaternion
class Feedback():
    def __init__(self):
        super(Feedback, self).__init__()
        self.feedback=np.zeros(4)
        self.feedback_gain=0.002
        self.faster_counter=0
        self.length_scale = 0.005
        self.correction_window = 300
        self.img_feedback_flag = 0
        self.spiral_flag = 0
        self.img_feedback_correction = 0
        self.gripper_feedback_correction = 0
        self.spiral_feedback_correction=0
        self.pause=False

        self.listener = Listener(on_press=self._on_press, on_release=self._on_release)
        self.listener.start()

    def _on_release(self, key):
        pass

    def _on_press(self, key):
        # self.get_logger().info(f"Event happened, user pressed {key}")
        # This function runs on the background and checks if a keyboard key was pressed
        if key == KeyCode.from_char('e'):
            self.end = True
        # Feedback for translate forward/backward
        if key == KeyCode.from_char('w'):
            self.feedback[0] = self.feedback_gain
        if key == KeyCode.from_char('s'):
            self.feedback[0] = -self.feedback_gain
        # Feedback for translate left/right
        if key == KeyCode.from_char('a'):
            self.feedback[1] = self.feedback_gain
        if key == KeyCode.from_char('d'):
            self.feedback[1] = -self.feedback_gain
        # Feedback for translate up/down
        if key == KeyCode.from_char('u'):
            self.feedback[2] = self.feedback_gain
        if key == KeyCode.from_char('j'):
            self.feedback[2] = -self.feedback_gain
        # Close/open gripper
        if key == KeyCode.from_char('c'):
            try:
                self.grip_value = 0
                self.grasp_gripper(self.grip_value)
                self.gripper_feedback_correction = 1
            except AttributeError:
                print("No robot available", flush=True)

        if key == KeyCode.from_char('o'):
            try:
                self.grip_value = self.grip_open_width
                self.move_gripper(self.grip_value)
                self.gripper_feedback_correction = 1
            except AttributeError:
                print("No robot available", flush=True)
        if key == KeyCode.from_char('f'):
            self.feedback[3] = 1
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
            quat_goal = list_2_quaternion(self.curr_ori_wxyz)
            goal = pos_quat_2_pose_st(self.curr_pos, quat_goal)

            self.move_to_pose_with_stampedpose(goal)
            
            self.set_stiffness(0, 0, 0, 50, 50, 50, 0)
            print("higher rotatioal stiffness")

        if key == KeyCode.from_char('n'):    
            self.set_stiffness(0, 0, 0, 0, 0, 0, 0)
            print("zero rotatioal stiffness")
        if key == Key.space:
            self.pause=not(self.pause)
            if self.pause==True:
                print("Recording paused")    
            else:
                print("Recording started again")  
        key=0

    def square_exp(self, ind_curr, ind_j):
        dist = np.sqrt((self.recorded_traj[0][ind_curr]-self.recorded_traj[0][ind_j])**2+(self.recorded_traj[1][ind_curr]-self.recorded_traj[1][ind_j])**2+(self.recorded_traj[2][ind_curr]-self.recorded_traj[2][ind_j])**2)
        sq_exp = np.exp(-dist**2/self.length_scale**2)
        return sq_exp    

    def correct(self):
        if np.sum(self.feedback[:3])!=0:
            for j in range(self.recorded_traj.shape[1]):
                x = self.feedback[0]*self.square_exp(self.time_index, j)
                y = self.feedback[1]*self.square_exp(self.time_index, j)
                z = self.feedback[2]*self.square_exp(self.time_index, j)

                self.recorded_traj[0][j] += x
                self.recorded_traj[1][j] += y
                self.recorded_traj[2][j] += z
        
        if self.img_feedback_correction:
            self.recorded_img_feedback_flag[0, self.time_index:] = self.img_feedback_flag

        if self.spiral_feedback_correction:
            self.recorded_spiral_flag[0, self.time_index:] = self.spiral_flag
        
        if self.gripper_feedback_correction:
            self.recorded_gripper[0, self.time_index:] = self.grip_value

        if self.feedback[3] != 0:
            self.faster_counter = 10
            
        if self.faster_counter > 0 and self.time_index != self.recorded_traj.shape[1]-1:
            self.faster_counter -= 1
            self.recorded_traj = np.delete(self.recorded_traj, self.time_index+1, 1)
            self.recorded_ori_wxyz = np.delete(self.recorded_ori_wxyz, self.time_index+1, 1)
            self.recorded_gripper = np.delete(self.recorded_gripper, self.time_index+1, 1)
            self.recorded_img = np.delete(self.recorded_img, self.time_index+1, 0)
            self.recorded_img_feedback_flag = np.delete(self.recorded_img_feedback_flag, self.time_index+1, 1)
            self.recorded_spiral_flag = np.delete(self.recorded_spiral_flag, self.time_index+1, 1)
                       
        self.feedback = np.zeros(4)
        self.img_feedback_correction = 0
        self.gripper_feedback_correction = 0
        self.spiral_feedback_correction = 0 

    

class FrankaOnPress():
    def __init__(self):
        super(FrankaOnPress, self).__init__()
        assert HAS_ROS, "ROS couldn't be imported"
        if not is_roscore_running(): return
        
        self.button_x_subscriber = rospy.Subscriber('/franka_buttons/x', Float32, self.cb_x, queue_size=10)
        self.button_y_subscriber = rospy.Subscriber('/franka_buttons/y', Float32, self.cb_y, queue_size=10)
        self.button_circle_subscriber = rospy.Subscriber('/franka_buttons/circle', Bool, self.cb_circle, queue_size=10)
        self.button_cross_subscriber = rospy.Subscriber('/franka_buttons/cross', Bool, self.cb_cross, queue_size=10)
        self.button_check_subscriber = rospy.Subscriber('/franka_buttons/check', Bool, self.cb_check, queue_size=10)

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



class RiskAwareFrankaButtons(FrankaOnPress):
    def __init__(self):
        super(RiskAwareFrankaButtons, self).__init__()
    def franka_on_press(self, key):
        if key == "check":
            self._on_press(KeyCode.from_char("t")) # safe
        elif key == "cross":
            self._on_press(KeyCode.from_char("r")) # danger
        elif key == "circle":
            self._on_press(KeyCode.from_char("q"))
        elif key == "x_positive":
            print("x=1 button have no mapping")
        elif key == "x_negative":
            print("x=-1 button have no mapping")
        elif key == "y_positive":
            print("y=1 button have no mapping")
        elif key == "y_negative":
            print("y=-1 button have no mapping")

    def franka_on_release(self, key):
        if key == "check":
            self._on_release(KeyCode.from_char("t")) # safe
        elif key == "cross":
            self._on_release(KeyCode.from_char("r")) # danger
        elif key == "circle":
            self._on_release(KeyCode.from_char("q"))
        elif key == "x_positive":
            print("x=1 button have no mapping")
        elif key == "x_negative":
            print("x=-1 button have no mapping")
        elif key == "y_positive":
            print("y=1 button have no mapping")
        elif key == "y_negative":
            print("y=-1 button have no mapping")



class RiskAwareFeedback(Feedback, RiskAwareFrankaButtons):
    def __init__(self, button_press_mode: str = "momentary"):
        super(RiskAwareFeedback, self).__init__()
        self.risk_flag = 0
        self.safe_flag = 0
        self.novelty_flag = 0
        self.recovery_phase = -1.

        try:
            self.button_press_mode
        except AttributeError:
            self.button_press_mode = button_press_mode


    def _on_press(self, key):
        if self.button_press_mode == 'toggle':
            self._on_press_toggle(key)
        elif self.button_press_mode == 'momentary':
            self._on_press_momentary(key)
        else: raise Exception()

    def _on_release(self, key):
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
                try:
                    trajectory_len = self.trajectory_len
                except AttributeError:
                    trajectory_len = 400
                self.target_time_index = int(fraction * trajectory_len)
                self.recovery_phase = fraction

        super()._on_press(key)

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
                try:
                    trajectory_len = self.trajectory_len
                except AttributeError:
                    trajectory_len = 400
                self.target_time_index = int(fraction * trajectory_len)
                self.recovery_phase = fraction

        super()._on_press(key)

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
    rospy.init_node('listener', anonymous=True)
    # fb = FrankaButtons()
    
    # raf = RiskAwareFeedback(button_press_mode="toggle")
    raf = RiskAwareFeedback(button_press_mode="momentary")
    input("Press enter to quit")