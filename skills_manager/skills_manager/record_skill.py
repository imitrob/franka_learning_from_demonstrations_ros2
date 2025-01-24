#!/usr/bin/env python3
"""
Recording trajectories and storing them into a databaseself.
"""
from skills_manager.lfd import LfD
import rclpy
from skills_manager.ros_param_manager import get_remote_parameter

def main():
    rclpy.init()
    try:
        lfd = LfD()
        lfd.start()

        lfd.declare_parameter('name_skill', "skill")
        name_skill = get_remote_parameter(lfd, "name_skill", server="recording_node")
        print(f"Recording skill: {name_skill}", flush=True)
        
        lfd.traj_rec()
        lfd.save(name_skill)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()