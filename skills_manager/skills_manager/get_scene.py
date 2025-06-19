#!/usr/bin/env python3
"""
Playback of trajectories and storing them into a databaseself.
"""
from skills_manager.lfd import LfD
import rclpy

from std_srvs.srv import Trigger

def main():
    rclpy.init()
    lfd = LfD()
    lfd.start()
    
    lfd.get_scene_client.call(Trigger.Request())

if __name__ == '__main__':
    main()