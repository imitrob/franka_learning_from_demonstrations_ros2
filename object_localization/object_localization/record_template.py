#!/usr/bin/env python3
"""Interactive CLI client for the persistent RecordTemplate action."""
import rclpy
from cv_bridge import CvBridge
from lfd_msgs.action import RecordTemplate
from lfd_msgs.srv import SubmitTemplateCrop
from rclpy.action import ActionClient
from rclpy.node import Node

from object_localization.gui import Template


def main():
    rclpy.init()
    node = Node("record_template_node")
    client = ActionClient(node, RecordTemplate, "/object_localization/record_template")
    crop_client = node.create_client(
        SubmitTemplateCrop, "/object_localization/record_template/submit_crop"
    )
    bridge = CvBridge()
    state = {"phase": "", "session": "", "image": None, "submitted": False}
    goal_handle = None

    def feedback(message):
        value = message.feedback
        if value.phase != state["phase"]:
            state["phase"] = value.phase
            print(value.phase, flush=True)
        if value.phase == "cropping" and value.preview.data:
            state["session"] = value.session_id
            state["image"] = bridge.imgmsg_to_cv2(value.preview, "bgr8")

    try:
        node.declare_parameter("name_template", "template")
        node.declare_parameter("overwrite_existing", False)
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("RecordTemplate action is unavailable; run localization_launch.py")
        if not crop_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("template crop service is unavailable")

        goal = RecordTemplate.Goal()
        goal.template_name = str(node.get_parameter("name_template").value)
        goal.overwrite_existing = bool(
            node.get_parameter("overwrite_existing").value
        )
        sent = client.send_goal_async(goal, feedback_callback=feedback)
        rclpy.spin_until_future_complete(node, sent)
        goal_handle = sent.result()
        if not goal_handle.accepted:
            raise RuntimeError("RecordTemplate request rejected")

        result = goal_handle.get_result_async()
        while rclpy.ok() and not result.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            if state["image"] is not None and not state["submitted"]:
                state["submitted"] = True
                crop = Template.select_crop(state["image"])
                if crop is None:
                    goal_handle.cancel_goal_async()
                    continue
                request = SubmitTemplateCrop.Request()
                request.session_id = state["session"]
                request.x_min, request.x_max, request.y_min, request.y_max = crop
                submitted = crop_client.call_async(request)
                rclpy.spin_until_future_complete(node, submitted)
                if not submitted.result().success:
                    raise RuntimeError(submitted.result().message)
        if result.done():
            print(result.result().result.message, flush=True)
    except KeyboardInterrupt:
        if goal_handle is not None:
            cancel = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=2.0)
    except Exception as exc:
        node.get_logger().error(str(exc))
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
