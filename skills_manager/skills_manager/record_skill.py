#!/usr/bin/env python3
"""Compatibility CLI client for the persistent RecordSkill action."""
import time

import rclpy
from lfd_msgs.action import RecordSkill
from lfd_msgs.srv import Heartbeat
from rclpy.action import ActionClient
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node("recording_node")
    client = ActionClient(node, RecordSkill, "/lfd/record_skill")
    heartbeat = node.create_client(Heartbeat, "/lfd/record_skill/heartbeat")
    state = {"recording_id": "", "phase": ""}
    goal_handle = None

    def feedback(message):
        value = message.feedback
        state["recording_id"] = value.recording_id
        if value.phase != state["phase"]:
            state["phase"] = value.phase
            print(value.phase, flush=True)

    try:
        node.declare_parameter("name_skill", "skill")
        node.declare_parameter("name_template", "")
        node.declare_parameter("homing", True)
        node.declare_parameter("overwrite_existing", False)
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("lfd_server RecordSkill action is unavailable")

        goal = RecordSkill.Goal()
        goal.skill_name = str(node.get_parameter("name_skill").value)
        goal.template_name = str(node.get_parameter("name_template").value)
        goal.home_before_recording = bool(node.get_parameter("homing").value)
        goal.overwrite_existing = bool(
            node.get_parameter("overwrite_existing").value
        )
        sent = client.send_goal_async(goal, feedback_callback=feedback)
        rclpy.spin_until_future_complete(node, sent)
        goal_handle = sent.result()
        if not goal_handle.accepted:
            raise RuntimeError("RecordSkill request rejected")

        result = goal_handle.get_result_async()
        next_heartbeat = 0.0
        while rclpy.ok() and not result.done():
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            if state["recording_id"] and now >= next_heartbeat:
                request = Heartbeat.Request()
                request.session_id = state["recording_id"]
                if heartbeat.service_is_ready():
                    heartbeat.call_async(request)
                next_heartbeat = now + 5.0
        if result.done():
            print(result.result().result.message, flush=True)
    except KeyboardInterrupt:
        if goal_handle is not None:
            cancel = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=2.0)
    except Exception as exc:
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
