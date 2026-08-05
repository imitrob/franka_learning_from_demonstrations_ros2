#!/usr/bin/env python3
"""
Recording trajectories and storing them into a databaseself.
"""
from skills_manager.lfd import LfD
import rclpy
from skills_manager.ros_param_manager import get_remote_parameter
from skills_manager.ros_param_manager import set_remote_parameters
from panda_control.home_pose import HOME_POSE

def main():
    rclpy.init()
    try:
        lfd = LfD()
        lfd.start()
        lfd.keyboard_start()
        lfd.frankabuttons_start()
        lfd.joy_start()
        # lfd.teleop_start()

        lfd.declare_parameter('name_skill', "no_skill_specified")
        lfd.declare_parameter('name_template', "")
        lfd.declare_parameter('move_start_flag', False)
        name_skill = get_remote_parameter(lfd, "name_skill", server="recording_node")
        name_template = get_remote_parameter(lfd, "name_template", server="recording_node")
        move_start_flag = get_remote_parameter(lfd, "move_start_flag", server="recording_node")

        print(f"Recording skill: {name_skill}", flush=True)
        print(f"First, localizing: {name_template}", flush=True)
        if move_start_flag:
            orientation_wxyz = HOME_POSE.orientation_wxyz
            set_remote_parameters(lfd, ["position_x", "position_y", "position_z", "orientation_x", "orientation_y", "orientation_z", "orientation_w"],
                [*HOME_POSE.position, *orientation_wxyz[1:], orientation_wxyz[0]],
                server="localizer_node")
            lfd.home_gripper(); lfd.move_template_start() # I need to always see both robot and gripper moving for sanity check
        lfd.localize(name_template)
        
        lfd.traj_rec()
        lfd.save(name_skill)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
