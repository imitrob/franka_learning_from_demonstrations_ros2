#!/usr/bin/env python3
"""Compatibility CLI client for the persistent HomeRobot action."""
import rclpy
from lfd_msgs.action import HomeRobot
from panda_control.home_pose import HOME_POSE
from rclpy.action import ActionClient
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node("homing_node")
    client = ActionClient(node, HomeRobot, "/lfd/home_robot")
    goal_handle = None
    try:
        node.declare_parameter("height", float(HOME_POSE.position[2]))
        node.declare_parameter("front_offset", float(HOME_POSE.position[0]))
        node.declare_parameter("side_offset", float(HOME_POSE.position[1]))
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("lfd_server HomeRobot action is unavailable")

        goal = HomeRobot.Goal()
        goal.height = float(node.get_parameter("height").value)
        goal.front_offset = float(node.get_parameter("front_offset").value)
        goal.side_offset = float(node.get_parameter("side_offset").value)
        sent = client.send_goal_async(
            goal, feedback_callback=lambda msg: print(msg.feedback.phase, flush=True)
        )
        rclpy.spin_until_future_complete(node, sent)
        goal_handle = sent.result()
        if not goal_handle.accepted:
            raise RuntimeError("HomeRobot request rejected: another operation is active")
        result = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result)
        print(result.result().result.message, flush=True)
    except KeyboardInterrupt:
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
    except Exception as exc:
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
