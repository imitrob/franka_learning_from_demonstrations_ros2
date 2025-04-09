#!/usr/bin/env python3
import rclpy
from panda_control import SpinPandaNode
from skills_manager.ros_param_manager import get_remote_parameters

def main():
    rclpy.init()
    panda=SpinPandaNode()
    panda.start()

    panda.declare_parameter('height', 0.4) # Rewritten for the values from launch file
    panda.declare_parameter('front_offset', 0.4) # Rewritten for the values from launch file
    panda.declare_parameter('side_offset', 0.4) # Rewritten for the values from launch file

    height = get_remote_parameters(panda, param_names=["height"], server="homing_node")[0]
    front_offset = get_remote_parameters(panda, param_names=["front_offset"], server="homing_node")[0]
    side_offset = get_remote_parameters(panda, param_names=["side_offset"], server="homing_node")[0]

    print("Height parameter: ", height, flush=True)
    print("Front offset parameter: ", front_offset, flush=True)
    print("Side offset parameter: ", side_offset, flush=True)
    
    panda.home(height=height , front_offset=front_offset, side_offset=side_offset)
    panda.offset_compensator(20)
    
    print("Done", flush=True)

if __name__ == "__main__":
    main()
