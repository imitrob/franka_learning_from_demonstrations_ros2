#!/usr/bin/env python3
from object_localization.gui import Template
import sys
import rclpy
from rclpy.node import Node

from skills_manager.ros_param_manager import extract_param_value

def main():
    rclpy.init()
    try:
        template = Template()
        param = template.panda.declare_parameter('template_name', "template")
        name_template, _ = extract_param_value(param.get_parameter_value())
        print("Recording template name: ", name_template, flush=True)
        template.record(name=name_template)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
