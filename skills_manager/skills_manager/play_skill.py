#!/usr/bin/env python3
"""Compatibility CLI client for the persistent ExecuteSkill action."""
import rclpy
from lfd_msgs.action import ExecuteSkill
from multi_modal_reasoning.skill_command import SkillCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_data.skill_part import SkillPart


def command_from_skill_name(name_skill: str, name_template: str = ""):
    part = SkillPart(name_skill)
    template = name_template.strip() or part.object
    if not part.action or not template:
        raise ValueError("name_skill must be <action>__<object>")
    if template != part.object:
        raise ValueError("name_template must match the object in name_skill")

    command = SkillCommand("", {})
    command.action = part.action
    # A part digit (put1__box) means the action is recorded in two parts, and
    # the server derives one part per object: same object twice replays both.
    command.objects = [part.object] * (2 if part.part else 1)
    return command


def main():
    rclpy.init()
    node = Node("execute_node")
    client = ActionClient(node, ExecuteSkill, "/lfd/execute_skill")
    goal_handle = None
    try:
        node.declare_parameter("name_skill", "skill")
        node.declare_parameter("name_template", "")
        node.declare_parameter("localize_box", True)  # legacy, localization is automatic
        command = command_from_skill_name(
            str(node.get_parameter("name_skill").value),
            str(node.get_parameter("name_template").value),
        )
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("lfd_server ExecuteSkill action is unavailable")

        goal = ExecuteSkill.Goal()
        goal.skill_command_json = command.to_json()
        sent = client.send_goal_async(
            goal,
            feedback_callback=lambda msg: print(msg.feedback.phase, flush=True),
        )
        rclpy.spin_until_future_complete(node, sent)
        goal_handle = sent.result()
        if not goal_handle.accepted:
            raise RuntimeError("ExecuteSkill request rejected")
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
