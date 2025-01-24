import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import numpy as np

class CustomTransformListener():
    def __init__(self):
        super(CustomTransformListener, self).__init__()
        self.transforms = {}  # Store transformations keyed by (parent, child)
        self.subscription_tf = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )
        self.subscription_tf_static = self.create_subscription(
            TFMessage,
            '/tf_static',
            self.tf_callback,
            10
        )

    def tf_callback(self, msg):
        for transform in msg.transforms:
            key = (transform.header.frame_id, transform.child_frame_id)
            self.transforms[key] = transform

    def __get_transform(self, parent_frame, child_frame):
        key = (parent_frame, child_frame)
        if key in self.transforms:
            return self.transforms[key]
        else:
            self.get_logger().warning(f"Transform {parent_frame} -> {child_frame} not found.")
            return None

    def lookup_relative_transform(self, source_frame, target_frame):
        source_to_target = self.__get_transform(source_frame, target_frame)
        if source_to_target:
            translation = source_to_target.transform.translation
            rotation = source_to_target.transform.rotation
            return translation, rotation
        return None, None


def main(args=None):
    rclpy.init(args=args)
    node = CustomTransformListener()

    try:
        # Replace with your frames
        source_frame = 'base_link'
        target_frame = 'camera_link'

        rclpy.spin_once(node, timeout_sec=0.1)

        translation, rotation = node.lookup_relative_transform(source_frame, target_frame)
        if translation and rotation:
            node.get_logger().info(
                f"Transform {source_frame} -> {target_frame}:\n"
                f"Translation: x={translation.x}, y={translation.y}, z={translation.z}\n"
                f"Rotation: x={rotation.x}, y={rotation.y}, z={rotation.z}, w={rotation.w}"
            )
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
