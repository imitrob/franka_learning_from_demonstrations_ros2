#!/usr/bin/env python


import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

import yaml
import object_localization

class StaticTFPublisher(Node):
    def __init__(self, tf_file: str):
        """Publishes TF that represents static transformations for given setup
        """        
        super(StaticTFPublisher, self).__init__('static_tf_pub')

        # Static Transform Broadcaster
        self.static_broadcasters = []

        with open(object_localization.package_path+"/config/"+tf_file) as f:
            tf_dict = yaml.safe_load(f)

        for tf_name, tf_values in tf_dict.items():
            self.static_broadcasters.append(StaticTransformBroadcaster(self))

            static_transform = TransformStamped()
            static_transform.header.stamp = self.get_clock().now().to_msg()
            static_transform.header.frame_id = "panda_hand"
            static_transform.child_frame_id = "camera_color_optical_frame"
            static_transform.transform.translation.x = tf_values['position'][0]
            static_transform.transform.translation.y = tf_values['position'][1]
            static_transform.transform.translation.z = tf_values['position'][2]
            static_transform.transform.rotation.x = tf_values['orientation'][0]
            static_transform.transform.rotation.y = tf_values['orientation'][1]
            static_transform.transform.rotation.z = tf_values['orientation'][2]
            static_transform.transform.rotation.w = tf_values['orientation'][3]
            self.static_broadcasters[-1].sendTransform(static_transform)
            self.static_transform = static_transform

            self.declare_parameter("position_x", tf_values['position'][0])
            self.declare_parameter("position_y", tf_values['position'][1])
            self.declare_parameter("position_z", tf_values['position'][2])
            self.declare_parameter("orientation_x", tf_values['orientation'][0])
            self.declare_parameter("orientation_y", tf_values['orientation'][1])
            self.declare_parameter("orientation_z", tf_values['orientation'][2])
            self.declare_parameter("orientation_w", tf_values['orientation'][3])

        self.timer = self.create_timer(0.5, self.send_tf)

    def send_tf(self):
        self.static_broadcasters[-1].sendTransform(self.static_transform)
        
            
def main():
    rclpy.init(args=None)
    node = StaticTFPublisher(tf_file="camera_transform.yaml")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()









