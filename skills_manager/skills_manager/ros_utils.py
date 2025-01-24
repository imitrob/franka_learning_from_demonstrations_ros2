
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading

from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np

class SpinningRosNode(Node):
    def __init__(self):
        super(SpinningRosNode, self).__init__(f"panda_node_{np.random.randint(100000)}") # node name replaced by launch description
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(self)
        spinning_thread = threading.Thread(target=executor.spin, daemon=True)
        spinning_thread.start()

        self.callback_group = ReentrantCallbackGroup()