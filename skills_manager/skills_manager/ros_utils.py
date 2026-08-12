
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, SingleThreadedExecutor
import threading

from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np

from skills_manager.ros_param_manager import get_remote_parameter, set_remote_parameter
from rclpy.exceptions import ParameterAlreadyDeclaredException
import rclpy

class SpinningRosNode(Node):
    def __init__(self):
        super(SpinningRosNode, self).__init__(f"panda_node_{np.random.randint(100000)}") # node name replaced by launch description
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(self)
        spinning_thread = threading.Thread(
            target=self._spin_forever, args=(executor,), daemon=True
        )
        spinning_thread.start()

        self.callback_group = ReentrantCallbackGroup()

        self.get_remote_parameter = get_remote_parameter
        self.set_remote_parameter = set_remote_parameter

    def _spin_forever(self, executor):
        """Keep serving after a callback raises.

        A single failing callback (a dead client that cannot receive its goal
        response, for instance) used to propagate out of executor.spin() and
        kill this thread, leaving the process alive but deaf: action servers
        stopped answering and clients hung forever with no log line.
        """
        while rclpy.ok():
            try:
                executor.spin()
                return  # clean shutdown
            except ExternalShutdownException:
                return
            except Exception as exc:  # noqa: BLE001 -- must survive any callback
                self.get_logger().error(f"Executor callback failed: {exc}")

    def declare_parameter_and_get(self, name, default_value):
        try:
            self.declare_parameter(name, default_value)
        except ParameterAlreadyDeclaredException:
            self.set_remote_parameter(self, name, default_value)
        return self.get_remote_parameter(self, name)



# class SpinningRosNode(Node):
#     def __init__(self):
#         super().__init__(f"panda_node_{np.random.randint(100000)}")

#         self._executor = SingleThreadedExecutor()
#         self._executor.add_node(self)

#         self._spin_thread = threading.Thread(target=self._spin, daemon=True)
#         self._spin_thread.start()

#         self.callback_group = ReentrantCallbackGroup()

#         self.get_remote_parameter = get_remote_parameter
#         self.set_remote_parameter = set_remote_parameter

#     def declare_parameter_and_get(self, name, default_value):
#         try:
#             self.declare_parameter(name, default_value)
#         except ParameterAlreadyDeclaredException:
#             self.set_remote_parameter(self, name, default_value)
#         return self.get_remote_parameter(self, name)

#     def _spin(self):
#         # Small timeout makes the thread yield regularly.
#         # 0.01 is a good starting point.
#         try:
#             while rclpy.ok():
#                 self._executor.spin_once(timeout_sec=0.01)
#         except ExternalShutdownException:
#             pass  # rclpy.shutdown() from the main thread ends the loop

