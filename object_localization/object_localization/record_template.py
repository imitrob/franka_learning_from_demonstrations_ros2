#!/usr/bin/env python3
from object_localization.gui import Template
import rclpy

from skills_manager.ros_param_manager import extract_param_value
from skills_manager.lfd import LfD
from skills_manager.ros_param_manager import set_remote_parameters

def main():
    rclpy.init()

    # lfd = LfD()
    # lfd.start()

    # If this flow moves the arm before recording again, source that target from
    # panda_control.home_pose.HOME_POSE, as the homing and localization flows do.

    # lfd.move_template_start()

    template = None
    try:
        template = Template()
        param = template.panda.declare_parameter('name_template', "template")
        name_template, _ = extract_param_value(param.get_parameter_value())
        print("Recording template name: ", name_template, flush=True)
        template.record(name=name_template)
    except KeyboardInterrupt:
        pass
    finally:
        # template.panda's background spin thread loops on rclpy.ok(); without
        # shutting down here it survives past main() and aborts mid-interpreter-exit.
        if template is not None:
            template.panda.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
