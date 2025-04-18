from panda_control.pose_transform_functions import orientation_2_quaternion, position_2_array, pos_quat_2_pose_st
import numpy as np

class Insertion():
    def __init__(self):
        super(Insertion, self).__init__()
    def spiral_search(self, goal, control_rate=30, force_min_exit=1): 
        # force_min_exit in Newton. If the verical force is lower than this value, the spiral search is considered successful
        r = self.create_rate(control_rate)
        max_spiral_time = 30 # seconds
        increase_radius_per_second = 0.0005 # meters, distance from center of the spiral after 1 second
        rounds_per_second = 1 # how many rounds does the spiral every second
        dt = 1. / control_rate
        goal_init = position_2_array(goal.pose.position)
        pos_init = self.curr_pos
        ori_quat = orientation_2_quaternion(goal.pose.orientation)
        goal_pose = pos_quat_2_pose_st(goal_init, ori_quat)
        time= 0
        spiral_success = False
        self.set_stiffness(self.K_pos, self.K_pos, self.K_pos/2, self.K_ori, self.K_ori, self.K_ori, 0) # get more compliant in z direction
        for _ in range(max_spiral_time * control_rate):   
            goal_pose.pose.position.x = pos_init[0] + np.cos(
                2 * np.pi *rounds_per_second*time) * increase_radius_per_second * time
            goal_pose.pose.position.y = pos_init[1] + np.sin(
                2 * np.pi *rounds_per_second* time) * increase_radius_per_second * time
            
            self.move_to_pose_with_stampedpose(goal_pose)

            if self.force.z <= force_min_exit: 
                spiral_success = True
                break
            time += dt
            r.sleep()
        self.set_stiffness(self.K_pos, self.K_pos, self.K_pos, self.K_ori, self.K_ori, self.K_ori, 0)    
        offset_correction = self.curr_pos - goal_init

        return spiral_success, offset_correction
