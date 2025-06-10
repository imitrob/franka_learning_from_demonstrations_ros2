#!/usr/bin/env python3
"""
Playback of trajectories and storing them into a databaseself.
"""
from skills_manager.lfd import LfD
import rclpy
from skills_manager.ros_param_manager import get_remote_parameters

def main():
    rclpy.init()
    lfd = LfD()
    lfd.start()

    lfd.declare_parameter('name_skill', "skill")
    lfd.declare_parameter('localize_box', True)
    lfd.declare_parameter('name_template', "sponge_template")
    name_skill = get_remote_parameters(lfd, param_names=["name_skill"], server=lfd.get_name())[0]
    localize_box = get_remote_parameters(lfd, param_names=["localize_box"], server=lfd.get_name())[0]
    name_template = get_remote_parameters(lfd, param_names=["name_template"], server=lfd.get_name())[0]
    print("Executing skill: ", name_skill, flush=True)
    print("Localize box: ", localize_box, flush=True)
    print("Localize template: ", name_template, flush=True)

    success = lfd.play_skill(name_skill, name_template, localize_box)

    save = True
    if save and success:
        lfd.save(name_skill+"_new")

if __name__ == '__main__':
    main()