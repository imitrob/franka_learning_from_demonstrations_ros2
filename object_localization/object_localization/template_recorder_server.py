#!/usr/bin/env python3
"""Headless template workflow server; never owns or imports Panda."""
import os
import shutil
import tempfile
import threading
import time
import uuid

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from lfd_msgs.action import RecordTemplate, ReserveRobot
from lfd_msgs.srv import Heartbeat, ReleaseReservation, SubmitTemplateCrop
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

import object_localization


COLOR_TOPIC = "/camera/color/image_raw"
DEPTH_TOPIC = "/camera/aligned_depth_to_color/image_raw"
POSE_TOPIC = "/panda/curr_pose"
RECORD_ACTION = "/object_localization/record_template"
RESERVE_ACTION = "/lfd/reserve_robot"


class _Canceled(Exception):
    pass


class TemplateRecorderServer(Node):
    def __init__(self):
        super().__init__("template_recorder_server")
        self.callback_group = ReentrantCallbackGroup()
        self.declare_parameter("camera_settle_seconds", 5.0)
        self.declare_parameter("capture_timeout", 15.0)
        self.declare_parameter("rgb_depth_slop", 0.15)

        self._lock = threading.Lock()
        self._frames = threading.Condition(self._lock)
        self._workflow_busy = False
        self._color = None
        self._depth = None
        self._pose = None
        self._active_session = ""
        self._crop = None
        self._crop_event = threading.Event()
        self._bridge = CvBridge()

        self.create_subscription(
            Image, COLOR_TOPIC, self._color_callback, 5,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Image, DEPTH_TOPIC, self._depth_callback, 5,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            PoseStamped, POSE_TOPIC, self._pose_callback, 5,
            callback_group=self.callback_group,
        )
        self._reserve_client = ActionClient(
            self, ReserveRobot, RESERVE_ACTION, callback_group=self.callback_group
        )
        self._heartbeat = self.create_client(
            Heartbeat, f"{RESERVE_ACTION}/heartbeat",
            callback_group=self.callback_group,
        )
        self._release = self.create_client(
            ReleaseReservation, f"{RESERVE_ACTION}/release",
            callback_group=self.callback_group,
        )
        self.create_service(
            SubmitTemplateCrop,
            f"{RECORD_ACTION}/submit_crop",
            self._submit_crop,
            callback_group=self.callback_group,
        )
        self._server = ActionServer(
            self,
            RecordTemplate,
            RECORD_ACTION,
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=self.callback_group,
        )

    def _goal(self, request):
        name = request.template_name.strip()
        if not name or os.path.basename(name) != name or name in (".", ".."):
            self.get_logger().warning("Rejecting invalid template name")
            return GoalResponse.REJECT
        target = os.path.join(object_localization.package_path, "cfg", name)
        if os.path.exists(target) and not request.overwrite_existing:
            self.get_logger().warning(f"Template {name!r} already exists")
            return GoalResponse.REJECT
        with self._lock:
            if self._workflow_busy:
                self.get_logger().warning("Another template workflow is active")
                return GoalResponse.REJECT
            self._workflow_busy = True
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        result = RecordTemplate.Result()
        session = uuid.uuid4().hex
        lease_id = ""
        reserve_handle = None
        lease_active = threading.Event()
        heartbeat_stop = threading.Event()
        try:
            with self._lock:
                self._active_session = session
                self._crop = None
                self._crop_event.clear()
            self._feedback(goal_handle, "waiting_for_server", session)
            if not self._reserve_client.wait_for_server(timeout_sec=10.0):
                raise RuntimeError("lfd_server ReserveRobot action is unavailable")

            ready = threading.Event()
            lease = {"id": ""}

            def reservation_feedback(message):
                phase = message.feedback.phase
                if message.feedback.lease_id:
                    lease["id"] = message.feedback.lease_id
                if phase == "homing":
                    self._feedback(goal_handle, "homing", session)
                elif phase == "ready":
                    ready.set()

            reserve_goal = ReserveRobot.Goal()
            reserve_goal.requester = f"record_template:{goal_handle.request.template_name}"
            sent = self._reserve_client.send_goal_async(
                reserve_goal, feedback_callback=reservation_feedback
            )
            self._wait(sent, goal_handle, 10.0)
            reserve_handle = sent.result()
            if not reserve_handle.accepted:
                raise RuntimeError("robot reservation rejected")
            reserve_result = reserve_handle.get_result_async()

            while not ready.wait(0.1):
                self._raise_if_canceled(goal_handle)
                if reserve_result.done():
                    raise RuntimeError(reserve_result.result().result.message)
            lease_id = lease["id"]
            lease_active.set()
            heartbeat_thread = threading.Thread(
                target=self._heartbeat_loop,
                args=(lease_id, lease_active, heartbeat_stop),
                daemon=True,
            )
            heartbeat_thread.start()

            self._feedback(goal_handle, "capturing", session)
            not_before = time.monotonic() + float(
                self.get_parameter("camera_settle_seconds").value
            )
            color_msg, depth_msg, pose_msg = self._capture(
                goal_handle, not_before
            )
            color = self._bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth = self._bridge.imgmsg_to_cv2(depth_msg, "passthrough")

            self._release_lease(lease_id)
            lease_active.clear()
            heartbeat_stop.set()
            self._wait(reserve_result, goal_handle, 5.0)

            self._feedback(goal_handle, "cropping", session, color_msg)
            while not self._crop_event.wait(0.1):
                self._raise_if_canceled(goal_handle)
            with self._lock:
                crop = self._crop

            self._feedback(goal_handle, "saving", session)
            saved = self._save(
                goal_handle.request.template_name.strip(),
                color,
                depth,
                pose_msg,
                crop,
                goal_handle.request.overwrite_existing,
            )
            result.message = f"Recorded template {goal_handle.request.template_name!r}"
            result.saved_path = saved
            goal_handle.succeed()
        except _Canceled:
            result.message = "Template recording canceled; snapshot discarded"
            goal_handle.canceled()
        except Exception as exc:
            result.message = f"Template recording failed: {exc}"
            self.get_logger().error(result.message)
            goal_handle.abort()
        finally:
            heartbeat_stop.set()
            if lease_active.is_set() and lease_id:
                try:
                    self._release_lease(lease_id)
                except Exception:
                    if reserve_handle is not None:
                        reserve_handle.cancel_goal_async()
            with self._lock:
                self._active_session = ""
                self._crop = None
                self._workflow_busy = False
            self._crop_event.clear()
        return result

    def _capture(self, goal_handle, not_before):
        deadline = time.monotonic() + float(
            self.get_parameter("capture_timeout").value
        ) + max(0.0, not_before - time.monotonic())
        slop = float(self.get_parameter("rgb_depth_slop").value)
        with self._frames:
            while True:
                self._raise_if_canceled(goal_handle)
                if time.monotonic() > deadline:
                    raise TimeoutError("fresh synchronized RGB-D frame unavailable")
                if self._color and self._depth and self._pose:
                    color_msg, color_arrival = self._color
                    depth_msg, depth_arrival = self._depth
                    if (
                        color_arrival >= not_before
                        and depth_arrival >= not_before
                        and abs(self._stamp(color_msg) - self._stamp(depth_msg)) <= slop
                    ):
                        return color_msg, depth_msg, self._pose[0]
                self._frames.wait(timeout=0.1)

    def _heartbeat_loop(self, lease_id, lease_active, stop):
        while not stop.wait(5.0) and lease_active.is_set() and rclpy.ok():
            request = Heartbeat.Request()
            request.session_id = lease_id
            if self._heartbeat.service_is_ready():
                self._heartbeat.call_async(request)

    def _release_lease(self, lease_id):
        if not self._release.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("reservation release service is unavailable")
        request = ReleaseReservation.Request()
        request.lease_id = lease_id
        future = self._release.call_async(request)
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done() or not future.result().success:
            raise RuntimeError("could not release robot reservation")

    def _submit_crop(self, request, response):
        with self._lock:
            valid_session = request.session_id == self._active_session
            color = None if self._color is None else self._color[0]
            width = 0 if color is None else color.width
            height = 0 if color is None else color.height
            valid_crop = (
                0 <= request.x_min < request.x_max <= width
                and 0 <= request.y_min < request.y_max <= height
            )
            if valid_session and valid_crop:
                self._crop = (
                    request.x_min, request.x_max, request.y_min, request.y_max
                )
                self._crop_event.set()
        response.success = valid_session and valid_crop
        response.message = "accepted" if response.success else "invalid session or crop"
        return response

    def _save(self, name, color, depth, pose, crop, overwrite):
        cfg = os.path.join(object_localization.package_path, "cfg")
        os.makedirs(cfg, exist_ok=True)
        target = os.path.join(cfg, name)
        if os.path.exists(target) and not overwrite:
            raise FileExistsError(f"template {name!r} already exists")
        temporary = tempfile.mkdtemp(prefix=f".{name}-", dir=cfg)
        backup = ""
        try:
            x_min, x_max, y_min, y_max = crop
            cropped = color[y_min:y_max, x_min:x_max]
            depth_crop = self._depth_crop(depth, color.shape, crop)
            depth_values = np.asarray(depth_crop).reshape(-1)
            depth_values = depth_values[
                np.isfinite(depth_values) & (depth_values > 0)
            ]
            if cropped.size == 0 or depth_values.size == 0:
                raise ValueError("crop contains no valid color/depth data")
            for filename, image in (
                ("full_image.png", color),
                ("template.png", cropped),
                ("depth.png", depth),
            ):
                if not cv2.imwrite(os.path.join(temporary, filename), image):
                    raise OSError(f"could not write {filename}")

            value = pose.pose
            params = {
                "template_path": f"/cfg/{name}/full_image.png",
                "crop": [x_min, x_max, y_min, y_max],
                "depth": float(np.median(depth_values)),
                "position": {
                    "x": float(value.position.x),
                    "y": float(value.position.y),
                    "z": float(value.position.z),
                },
                "orientation": {
                    "w": float(value.orientation.w),
                    "x": float(value.orientation.x),
                    "y": float(value.orientation.y),
                    "z": float(value.orientation.z),
                },
            }
            with open(os.path.join(temporary, "params.yaml"), "w") as stream:
                yaml.safe_dump(params, stream)
            with open(os.path.join(temporary, "params.yaml")) as stream:
                yaml.safe_load(stream)

            if os.path.exists(target):
                backup = f"{target}.backup-{uuid.uuid4().hex}"
                os.replace(target, backup)
            try:
                os.replace(temporary, target)
                temporary = ""
            except Exception:
                if backup:
                    os.replace(backup, target)
                    backup = ""
                raise
            if backup:
                shutil.rmtree(backup)
            return target
        finally:
            if temporary and os.path.isdir(temporary):
                shutil.rmtree(temporary)

    @staticmethod
    def _depth_crop(depth, color_shape, crop):
        """Map an RGB crop into depth coordinates when resolutions differ."""
        x_min, x_max, y_min, y_max = crop
        color_height, color_width = color_shape[:2]
        depth_height, depth_width = depth.shape[:2]
        if (color_height, color_width) != (depth_height, depth_width):
            x_min = int(np.floor(x_min * depth_width / color_width))
            x_max = int(np.ceil(x_max * depth_width / color_width))
            y_min = int(np.floor(y_min * depth_height / color_height))
            y_max = int(np.ceil(y_max * depth_height / color_height))
        x_min = max(0, min(x_min, depth_width))
        x_max = max(0, min(x_max, depth_width))
        y_min = max(0, min(y_min, depth_height))
        y_max = max(0, min(y_max, depth_height))
        return depth[y_min:y_max, x_min:x_max]

    def _feedback(self, goal_handle, phase, session, preview=None):
        feedback = RecordTemplate.Feedback()
        feedback.phase = phase
        feedback.session_id = session
        if preview is not None:
            feedback.preview = preview
        if goal_handle.is_active:
            goal_handle.publish_feedback(feedback)

    def _wait(self, future, goal_handle, timeout):
        deadline = time.monotonic() + timeout
        while not future.done():
            self._raise_if_canceled(goal_handle)
            if time.monotonic() > deadline:
                raise TimeoutError("ROS operation timed out")
            time.sleep(0.05)

    @staticmethod
    def _raise_if_canceled(goal_handle):
        if goal_handle.is_cancel_requested:
            raise _Canceled()

    @staticmethod
    def _stamp(message):
        return message.header.stamp.sec + message.header.stamp.nanosec / 1e9

    def _color_callback(self, message):
        with self._frames:
            self._color = (message, time.monotonic())
            self._frames.notify_all()

    def _depth_callback(self, message):
        with self._frames:
            self._depth = (message, time.monotonic())
            self._frames.notify_all()

    def _pose_callback(self, message):
        with self._frames:
            self._pose = (message, time.monotonic())
            self._frames.notify_all()


def main():
    rclpy.init()
    node = TemplateRecorderServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
