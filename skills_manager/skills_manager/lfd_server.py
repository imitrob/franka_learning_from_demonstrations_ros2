#!/usr/bin/env python3
"""Persistent single-owner server for every operation that moves Panda."""
import math
import os
import queue
import threading
import time
import traceback
import uuid

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from lfd_msgs.action import ExecuteSkill, HomeRobot, RecordSkill, ReserveRobot
from lfd_msgs.msg import OperationStatus
from lfd_msgs.srv import (
    FinishRecording,
    Heartbeat,
    ReleaseReservation,
    SetTemplate,
)
from multi_modal_reasoning.skill_command import SkillCommand
from panda_control.home_pose import HOME_POSE
from panda_control.panda import MotionCanceled, MotionError
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from trajectory_data.skill_part import SkillPart

from skills_manager.lfd import LfD


EXECUTE_ACTION = "/lfd/execute_skill"
HOME_ACTION = "/lfd/home_robot"
RECORD_ACTION = "/lfd/record_skill"
RESERVE_ACTION = "/lfd/reserve_robot"
STATUS_TOPIC = "/lfd/operation_status"
SKILL_COMMAND_TOPIC = "/hri/skill_command"
STOP_ACTION = "stop"
HOME_TOLERANCE = 0.05
HOME_ORIENTATION_TOLERANCE = 0.1
RECORD_HEARTBEAT_TIMEOUT = 30.0
LEASE_TIMEOUT = 30.0


_OperationCanceled = MotionCanceled


class _OperationTimedOut(Exception):
    pass


class LfDServer(LfD):
    """Own one Panda and admit exactly one robot operation at a time."""

    def __init__(self):
        super().__init__()

        self._state_lock = threading.Lock()
        self._operation_mode = OperationStatus.IDLE
        self._operation_id = ""
        self._operation_target = ""
        self._operation_phase = "idle"
        self._operation_message = ""
        self._active_goal = None
        self._active_action_name = ""

        self._stop_inflight = 0
        self._deferred_stop_goals = []
        self._stop_errors = {}
        self._stop_outcomes = {}
        self._deferred_stop_ids = set()
        self._cancel_sent = False
        self._cancel_error = ""

        self._record_finish = threading.Event()
        self._recording_id = ""
        self._record_deadline = 0.0
        self._record_timed_out = False
        self._lease_released = threading.Event()
        self._lease_id = ""
        self._lease_deadline = 0.0

        # Tk widgets belong to main(); executor workers only enqueue changes.
        self._signalizer_states = queue.SimpleQueue()

        self.declare_parameter("home_tolerance", HOME_TOLERANCE)
        self.declare_parameter(
            "home_orientation_tolerance", HOME_ORIENTATION_TOLERANCE
        )
        self.declare_parameter(
            "record_heartbeat_timeout", RECORD_HEARTBEAT_TIMEOUT
        )
        self.declare_parameter("template_lease_timeout", LEASE_TIMEOUT)

        status_qos = QoSProfile(
            depth=10,  # a fast phase (failed -> homing) must not overwrite its predecessor
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
            OperationStatus, STATUS_TOPIC, status_qos
        )

        self._execute_server = ActionServer(
            self,
            ExecuteSkill,
            EXECUTE_ACTION,
            execute_callback=self._execute_callback,
            goal_callback=self._execute_goal_callback,
            handle_accepted_callback=self._execute_accepted_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )
        self._home_server = ActionServer(
            self,
            HomeRobot,
            HOME_ACTION,
            execute_callback=self._home_callback,
            goal_callback=self._home_goal_callback,
            handle_accepted_callback=lambda goal: self._operation_accepted(
                goal, HOME_ACTION
            ),
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )
        self._record_server = ActionServer(
            self,
            RecordSkill,
            RECORD_ACTION,
            execute_callback=self._record_callback,
            goal_callback=self._record_goal_callback,
            handle_accepted_callback=lambda goal: self._operation_accepted(
                goal, RECORD_ACTION
            ),
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )
        self._reserve_server = ActionServer(
            self,
            ReserveRobot,
            RESERVE_ACTION,
            execute_callback=self._reserve_callback,
            goal_callback=self._reserve_goal_callback,
            handle_accepted_callback=lambda goal: self._operation_accepted(
                goal, RESERVE_ACTION
            ),
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )

        self._execute_client = ActionClient(
            self, ExecuteSkill, EXECUTE_ACTION, callback_group=self.callback_group
        )
        self._cancel_clients = {
            name: self.create_client(
                CancelGoal,
                f"{name}/_action/cancel_goal",
                callback_group=self.callback_group,
            )
            for name in (EXECUTE_ACTION, HOME_ACTION, RECORD_ACTION, RESERVE_ACTION)
        }
        self.create_service(
            FinishRecording,
            f"{RECORD_ACTION}/finish",
            self._finish_recording_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            Heartbeat,
            f"{RECORD_ACTION}/heartbeat",
            self._record_heartbeat_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            Heartbeat,
            f"{RESERVE_ACTION}/heartbeat",
            self._lease_heartbeat_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            ReleaseReservation,
            f"{RESERVE_ACTION}/release",
            self._release_reservation_callback,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            SKILL_COMMAND_TOPIC,
            self._skill_command_callback,
            10,
            callback_group=self.callback_group,
        )
        self._publish_operation_status()

    # --- shared admission/state -----------------------------------------

    def _reserve_operation(self, mode, target, action_name):
        with self._state_lock:
            if self._operation_mode != OperationStatus.IDLE or self._stop_inflight:
                self.get_logger().warning(
                    f"Rejecting {target!r}: robot operation is already active"
                )
                return GoalResponse.REJECT
            try:
                self.begin_motion()
            except MotionError as exc:
                self.get_logger().warning(f"Rejecting {target!r}: {exc}")
                return GoalResponse.REJECT
            self._operation_mode = mode
            self._operation_id = uuid.uuid4().hex
            self._operation_target = target
            self._operation_phase = "accepted"
            self._operation_message = ""
            self._active_goal = None
            self._active_action_name = action_name
            self._cancel_sent = False
            self._cancel_error = ""
            if mode == OperationStatus.RECORDING_SKILL:
                self._recording_id = self._operation_id
                self._record_deadline = (
                    time.monotonic()
                    + float(self.get_parameter("record_heartbeat_timeout").value)
                )
                self._record_finish.clear()
                self._record_timed_out = False
            elif mode == OperationStatus.CAPTURING_TEMPLATE:
                self._lease_id = self._operation_id
                self._lease_deadline = 0.0
                self._lease_released.clear()
        self._publish_operation_status()
        return GoalResponse.ACCEPT

    def _operation_accepted(self, goal_handle, action_name):
        with self._state_lock:
            self._active_goal = goal_handle
            self._active_action_name = action_name
            stop_is_waiting = self._stop_inflight > 0
        goal_handle.execute()
        if stop_is_waiting:
            self._request_active_cancel()

    def _cancel_callback(self, goal_handle):
        if hasattr(goal_handle.request, "skill_command_json"):
            try:
                if self._task_from_request(goal_handle.request).action == STOP_ACTION:
                    return CancelResponse.REJECT
            except ValueError:
                return CancelResponse.REJECT
        with self._state_lock:
            if self._operation_mode == OperationStatus.IDLE:
                return CancelResponse.REJECT
            if self._active_goal is None:
                self._active_goal = goal_handle
            elif self._active_goal is not goal_handle:
                return CancelResponse.REJECT
        try:
            self.stop()
            self.stop_gripper()
        except Exception as exc:
            self.get_logger().warning(f"Could not stop canceled motion: {exc}")
        return CancelResponse.ACCEPT

    def _set_operation_phase(self, phase, message=""):
        with self._state_lock:
            self._operation_phase = phase
            self._operation_message = message
        self._publish_operation_status()

    def _publish_operation_status(self):
        if not hasattr(self, "_status_pub"):
            return
        with self._state_lock:
            message = OperationStatus()
            message.mode = self._operation_mode
            message.operation_id = self._operation_id
            message.target = self._operation_target
            message.phase = self._operation_phase
            message.message = self._operation_message
        self._status_pub.publish(message)

    def _finish_operation(self, cleanup_error="", outcome="completed"):
        # The producer has unwound before admission is released.
        if outcome != "completed":
            try:
                self.stop()
                self.stop_gripper()
                self.wait_for_hold()
            except Exception as exc:
                cleanup_error = cleanup_error or str(exc)
        with self._state_lock:
            stop_error = cleanup_error or self._cancel_error
            deferred = list(self._deferred_stop_goals)
            self._deferred_stop_goals.clear()
            for stop_goal in deferred:
                stop_id = self._goal_id(stop_goal)
                self._stop_errors[stop_id] = stop_error
                self._stop_outcomes[stop_id] = outcome
            self._operation_mode = OperationStatus.IDLE
            self._operation_id = ""
            self._operation_target = ""
            self._operation_phase = "idle" if not cleanup_error else "failed"
            self._operation_message = cleanup_error
            self._active_goal = None
            self._active_action_name = ""
            self._recording_id = ""
            self._record_deadline = 0.0
            self._lease_id = ""
            self._lease_deadline = 0.0
            self._cancel_sent = False
            self._cancel_error = ""
        self._publish_operation_status()
        for stop_goal in deferred:
            stop_goal.execute()

    # --- ExecuteSkill and global stop -----------------------------------

    def _execute_goal_callback(self, request):
        try:
            task = self._task_from_request(request)
            self._validate_supported_task(task)
        except ValueError as exc:
            self.get_logger().warning(f"Rejecting skill task: {exc}")
            return GoalResponse.REJECT

        if task.action == STOP_ACTION:
            with self._state_lock:
                self._stop_inflight += 1
                if self._operation_mode != OperationStatus.IDLE:
                    self.stop()
            return GoalResponse.ACCEPT
        return self._reserve_operation(
            OperationStatus.EXECUTING, task.command, EXECUTE_ACTION
        )

    def _execute_accepted_callback(self, goal_handle):
        task = self._task_from_request(goal_handle.request)
        if task.action != STOP_ACTION:
            self._operation_accepted(goal_handle, EXECUTE_ACTION)
            return

        execute_now = False
        request_cancel = False
        with self._state_lock:
            if self._operation_mode != OperationStatus.IDLE:
                self._deferred_stop_goals.append(goal_handle)
                self._deferred_stop_ids.add(self._goal_id(goal_handle))
                request_cancel = self._active_goal is not None
            else:
                execute_now = True
        if request_cancel:
            self._request_active_cancel()
        if execute_now:
            goal_handle.execute()

    def _execute_callback(self, goal_handle):
        task = self._task_from_request(goal_handle.request)
        if task.action == STOP_ACTION:
            return self._execute_stop(goal_handle)
        return self._execute_task(goal_handle, task)

    def _skill_command_callback(self, message):
        goal = ExecuteSkill.Goal()
        goal.skill_command_json = message.data
        try:
            future = self._execute_client.send_goal_async(goal)
            future.add_done_callback(self._topic_goal_response)
        except Exception as exc:
            self.get_logger().error(f"Could not forward skill command: {exc}")

    def _topic_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Skill command action request failed: {exc}")
            return
        if not goal_handle.accepted:
            self.get_logger().warning("Skill command rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._topic_result)

    def _topic_result(self, future):
        try:
            wrapped = future.result()
        except Exception as exc:
            self.get_logger().error(f"Skill command result failed: {exc}")
            return
        log = self.get_logger().info
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            log = self.get_logger().warning
        elif wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            log = self.get_logger().error
        log(wrapped.result.message)

    def _execute_task(self, goal_handle, task):
        completed = []
        cleanup_error = ""
        outcome = "failed"
        cleaning_up = False
        result = ExecuteSkill.Result()
        try:
            try:
                parts = self.parts_for_task(task)
                self.end = False
                self._start_inputs()
                self._queue_signalizer("executing")
                self._raise_if_canceled(goal_handle)
                self._set_operation_phase("homing")
                self._publish_feedback(goal_handle, "homing", parts)
                self._release_home()
                self._raise_if_canceled(goal_handle)
                self._set_operation_phase("validating")
                self._validate_parts(goal_handle, parts)
                self._raise_if_canceled(goal_handle)
                for index, part in enumerate(parts, start=1):
                    if index > 1:
                        self._set_operation_phase("homing")
                        self._publish_feedback(goal_handle, "homing", parts, index=index, part=part)
                        self._carry_home()
                        self._raise_if_canceled(goal_handle)
                    self._set_operation_phase("localizing", part.name)
                    self._publish_feedback(goal_handle, "localizing", parts, index=index, part=part)
                    self._localize_part(part)
                    self._raise_if_canceled(goal_handle)
                    self._set_operation_phase("executing", part.name)
                    self._execute_part(goal_handle, parts, index, part)
                    completed.append(part.name)
                    self._raise_if_canceled(goal_handle)
                cleaning_up = True
                self._set_operation_phase("homing")
                self._publish_feedback(goal_handle, "homing", parts)
                self._release_home()
                self._raise_if_canceled(goal_handle)
                self._publish_feedback(goal_handle, "idle", parts, progress=1.0)
                goal_handle.succeed()
                outcome = "completed"
                result.message = self._result_message(task, "completed", completed)
            except Exception as exc:
                canceled = isinstance(exc, MotionCanceled) or goal_handle.is_cancel_requested
                if cleaning_up and not canceled:
                    cleanup_error = str(exc)
                outcome = "canceled" if canceled else "failed"
                detail = "canceled" if canceled else (
                    f"cleanup failed: {exc}" if cleaning_up else f"failed: {exc}")
                result.message = self._result_message(task, detail, completed)
                self.get_logger().warning(result.message)
                self._set_operation_phase(outcome, result.message)
                # A latched stop can reach here before ROS finishes its cancel transition.
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
            finally:
                self._stop_inputs()
                self._queue_signalizer("idle")
            result.completed_parts = completed
            return result
        finally:
            self._finish_operation(cleanup_error, outcome)

    def _execute_stop(self, goal_handle):
        goal_id = self._goal_id(goal_handle)
        with self._state_lock:
            error = self._stop_errors.pop(goal_id, "")
            outcome = self._stop_outcomes.pop(goal_id, "")
            waited = goal_id in self._deferred_stop_ids
            self._deferred_stop_ids.discard(goal_id)

        result = ExecuteSkill.Result()
        result.completed_parts = []
        self._publish_feedback(goal_handle, "idle", [], progress=1.0)
        try:
            if error:
                result.message = f"Stop completed with an error: {error}"
                goal_handle.abort()
            else:
                result.message = (
                    "No active operation; stop is a no-op"
                    if not waited
                    else f"Active operation {outcome or 'ended'}"
                )
                goal_handle.succeed()
        finally:
            with self._state_lock:
                self._stop_inflight -= 1
        return result

    # --- HomeRobot -------------------------------------------------------

    def _home_goal_callback(self, request):
        values = (request.height, request.front_offset, request.side_offset)
        if not all(math.isfinite(value) for value in values):
            return GoalResponse.REJECT
        return self._reserve_operation(
            OperationStatus.HOMING, "home", HOME_ACTION
        )

    def _home_callback(self, goal_handle):
        result = HomeRobot.Result()
        outcome = "failed"
        error = ""
        try:
            self._raise_if_canceled(goal_handle)
            self._set_operation_phase("homing")
            feedback = HomeRobot.Feedback()
            feedback.phase = "homing"
            goal_handle.publish_feedback(feedback)
            request = goal_handle.request
            self.home(
                height=request.height,
                front_offset=request.front_offset,
                side_offset=request.side_offset,
            )
            self.offset_compensator(20)
            self._raise_if_canceled(goal_handle)
            result.message = "Robot homed"
            goal_handle.succeed()
            outcome = "completed"
        except _OperationCanceled:
            result.message = "Homing canceled"
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            outcome = "canceled"
        except Exception as exc:
            error = str(exc)
            result.message = f"Homing failed: {exc}"
            goal_handle.abort()
        finally:
            try:
                self._queue_signalizer("idle")
            finally:
                self._finish_operation(error, outcome)
        return result

    # --- RecordSkill -----------------------------------------------------

    @staticmethod
    def _recording_part(request):
        raw = request.skill_name.strip()
        part = SkillPart(raw)
        if not raw or part.name != raw.removesuffix(".npz"):
            raise ValueError("invalid skill name")
        if part.is_variant or not part.action or not part.object:
            raise ValueError("skill name must be <action>__<object>")
        return part

    def _record_goal_callback(self, request):
        try:
            part = self._recording_part(request)
            if part.archive_path() and not request.overwrite_existing:
                try:
                    part.validate_archive()
                except FileNotFoundError:
                    pass
                else:
                    raise ValueError(f"skill {part.name!r} already exists")
        except ValueError as exc:
            self.get_logger().warning(f"Rejecting recording: {exc}")
            return GoalResponse.REJECT
        return self._reserve_operation(
            OperationStatus.RECORDING_SKILL, part.name, RECORD_ACTION
        )

    def _record_callback(self, goal_handle):
        result = RecordSkill.Result()
        part = self._recording_part(goal_handle.request)
        template = goal_handle.request.template_name.strip() or part.object
        outcome = "failed"
        error = ""
        saved = ""

        self.end = False
        try:
            self._start_inputs()
            self._raise_if_canceled(goal_handle)
            self._record_feedback(goal_handle, "homing")
            if goal_handle.request.home_before_recording:
                self.home()
                self.offset_compensator(20)
            self._raise_record_stop(goal_handle)
            if self._record_finish_requested():
                outcome = "completed"
                return self._finish_recording_early(goal_handle, result)

            self._record_feedback(goal_handle, "localizing")
            self._validate_template(template)
            if self.localize(template) is False:
                raise RuntimeError(f"{template} not found")
            self._raise_record_stop(goal_handle)
            if self._record_finish_requested():
                outcome = "completed"
                return self._finish_recording_early(goal_handle, result)

            started = self.traj_rec(
                should_stop=lambda: self._record_should_stop(goal_handle),
                on_phase=lambda phase: self._record_phase(goal_handle, phase),
                signalize=False,
            )
            self._raise_record_stop(goal_handle)
            if not started:
                outcome = "completed"
                return self._finish_recording_early(goal_handle, result)

            self._record_feedback(goal_handle, "saving")
            self.save(part.name, overwrite=goal_handle.request.overwrite_existing)
            saved = part.archive_path()
            result.message = f"Recorded {part.name!r}"
            result.saved_path = saved
            goal_handle.succeed()
            outcome = "completed"
            return result
        except _OperationCanceled:
            result.message = "Recording canceled; demonstration discarded"
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            outcome = "canceled"
        except _OperationTimedOut:
            result.message = "Recording client heartbeat timed out; discarded"
            goal_handle.abort()
        except Exception as exc:
            error = str(exc)
            result.message = f"Recording failed: {exc}"
            self.get_logger().error(f"Recording failed:\n{traceback.format_exc()}")
            self._set_operation_phase("failed", error)
            goal_handle.abort()
        finally:
            try:
                self._stop_inputs()
                self._restore_normal_stiffness()
                self._queue_signalizer("idle")
            finally:
                self._finish_operation(error, outcome)
        return result

    def _finish_recording_early(self, goal_handle, result):
        result.message = "Recording ended before a demonstration started; nothing saved"
        result.saved_path = ""
        goal_handle.succeed()
        return result

    def _record_phase(self, goal_handle, phase):
        self._queue_signalizer("ready" if phase == "ready" else "recording")
        self._record_feedback(goal_handle, phase)

    def _record_feedback(self, goal_handle, phase):
        self._set_operation_phase(phase)
        feedback = RecordSkill.Feedback()
        feedback.phase = phase
        feedback.recording_id = self._recording_id
        if goal_handle.is_active:
            goal_handle.publish_feedback(feedback)

    def _record_finish_requested(self):
        return self.end or self._record_finish.is_set()

    def _record_should_stop(self, goal_handle):
        if goal_handle.is_cancel_requested or not rclpy.ok():
            return True
        if self._record_finish_requested():
            return True
        with self._state_lock:
            timed_out = time.monotonic() > self._record_deadline
            self._record_timed_out = self._record_timed_out or timed_out
        return timed_out

    def _raise_record_stop(self, goal_handle):
        if goal_handle.is_cancel_requested or not rclpy.ok():
            raise _OperationCanceled("canceled")
        with self._state_lock:
            timed_out = self._record_timed_out or (
                self._record_deadline and time.monotonic() > self._record_deadline
            )
            self._record_timed_out = bool(timed_out)
        if timed_out:
            raise _OperationTimedOut()

    def _finish_recording_callback(self, request, response):
        with self._state_lock:
            valid = (
                self._operation_mode == OperationStatus.RECORDING_SKILL
                and request.recording_id == self._recording_id
            )
        if valid:
            self._record_finish.set()
            self.end = True
        response.success = valid
        response.message = "finish requested" if valid else "recording is not active"
        return response

    def _record_heartbeat_callback(self, request, response):
        with self._state_lock:
            valid = (
                self._operation_mode == OperationStatus.RECORDING_SKILL
                and request.session_id == self._recording_id
            )
            if valid:
                self._record_deadline = (
                    time.monotonic()
                    + float(self.get_parameter("record_heartbeat_timeout").value)
                )
        response.success = valid
        response.message = "renewed" if valid else "recording is not active"
        return response

    # --- camera-agnostic template reservation --------------------------

    def _reserve_goal_callback(self, request):
        requester = request.requester.strip()
        if not requester:
            return GoalResponse.REJECT
        return self._reserve_operation(
            OperationStatus.CAPTURING_TEMPLATE, requester, RESERVE_ACTION
        )

    def _reserve_callback(self, goal_handle):
        result = ReserveRobot.Result()
        outcome = "failed"
        error = ""
        try:
            self._raise_if_canceled(goal_handle)
            self._reserve_feedback(goal_handle, "homing")
            self.home()
            self.offset_compensator(20)
            self._raise_if_canceled(goal_handle)

            with self._state_lock:
                self._lease_deadline = (
                    time.monotonic()
                    + float(self.get_parameter("template_lease_timeout").value)
                )
            self._reserve_feedback(goal_handle, "ready")
            while rclpy.ok() and not self._lease_released.wait(0.1):
                self._raise_if_canceled(goal_handle)
                with self._state_lock:
                    expired = time.monotonic() > self._lease_deadline
                if expired:
                    raise _OperationTimedOut()

            self._raise_if_canceled(goal_handle)
            result.message = "Template reservation released"
            goal_handle.succeed()
            outcome = "completed"
        except _OperationCanceled:
            result.message = "Template reservation canceled"
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            outcome = "canceled"
        except _OperationTimedOut:
            result.message = "Template reservation timed out"
            goal_handle.abort()
            outcome = "timed_out"
        except Exception as exc:
            error = str(exc)
            result.message = f"Template reservation failed: {exc}"
            goal_handle.abort()
        finally:
            try:
                self._queue_signalizer("idle")
            finally:
                self._finish_operation(error, outcome)
        return result

    def _reserve_feedback(self, goal_handle, phase):
        self._set_operation_phase(phase)
        feedback = ReserveRobot.Feedback()
        feedback.phase = phase
        feedback.lease_id = self._lease_id
        with self._state_lock:
            remaining = max(0.0, self._lease_deadline - time.monotonic())
        feedback.expires_at = (
            self.get_clock().now() + Duration(seconds=remaining)
        ).to_msg()
        if goal_handle.is_active:
            goal_handle.publish_feedback(feedback)

    def _lease_heartbeat_callback(self, request, response):
        with self._state_lock:
            valid = (
                self._operation_mode == OperationStatus.CAPTURING_TEMPLATE
                and request.session_id == self._lease_id
                and self._lease_deadline > 0.0
            )
            if valid:
                self._lease_deadline = (
                    time.monotonic()
                    + float(self.get_parameter("template_lease_timeout").value)
                )
        response.success = valid
        response.message = "renewed" if valid else "reservation is not ready"
        return response

    def _release_reservation_callback(self, request, response):
        with self._state_lock:
            valid = (
                self._operation_mode == OperationStatus.CAPTURING_TEMPLATE
                and request.lease_id == self._lease_id
            )
        if valid:
            self._lease_released.set()
        response.success = valid
        response.message = "released" if valid else "reservation is not active"
        return response

    # --- shared robot helpers -------------------------------------------

    def _validate_parts(self, goal_handle, parts):
        self._publish_feedback(goal_handle, "validating", parts)
        for index, part in enumerate(parts, start=1):
            self._raise_if_canceled(goal_handle)
            self._publish_feedback(
                goal_handle, "validating", parts, index=index, part=part
            )
            try:
                part.validate_archive()
            except FileNotFoundError:
                raise RuntimeError(f"{part.action} {part.object} not learned yet")
            self._validate_template(part.object)

    def _validate_template(self, template):
        if not self.set_localizer_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("set_localizer service is unavailable")
        response = self.call_motion_service(
            self.set_localizer_client, SetTemplate.Request(template_name=template)
        )
        if response is None or not response.success:
            raise ValueError(f"localization template {template!r} is unavailable")

    def _localize_part(self, part):
        if self.localize(part.object) is False:
            raise RuntimeError(f"{part.object} not found")

    def _execute_part(self, goal_handle, parts, index, part):
        self.load(part.name)
        self.player_init()
        self._publish_feedback(
            goal_handle, "executing", parts, index=index, part=part
        )
        while self.time_index < self.loaded_trajectory_len:
            self._raise_if_canceled(goal_handle)
            self.player_step()
            self._publish_feedback(
                goal_handle,
                "executing",
                parts,
                index=index,
                part=part,
                progress=self.time_phase,
            )

    @staticmethod
    def parts_for_task(task):
        if len(task.objects) == 1:
            return [SkillPart(f"{task.action}{SkillPart.SEP}{task.objects[0]}")]
        return [
            SkillPart(f"{task.action}{index}{SkillPart.SEP}{obj}")
            for index, obj in enumerate(task.objects, start=1)
        ]

    @staticmethod
    def _validate_supported_task(task):
        if not task.action.strip():
            raise ValueError("action is empty")
        if task.parameters:
            raise ValueError("parameters are not supported yet")
        if task.action == STOP_ACTION:
            if task.objects:
                raise ValueError("stop takes no objects")
            return
        if len(task.objects) not in (1, 2):
            raise ValueError("v1 tasks require one or two objects")
        if any(not obj.strip() for obj in task.objects):
            raise ValueError("object names cannot be empty")

    @staticmethod
    def _task_from_request(request):
        return SkillCommand.from_json(request.skill_command_json)

    def _raise_if_canceled(self, goal_handle):
        self.check_motion()
        if not goal_handle.is_cancel_requested:
            return
        try:
            self.stop()
            self.stop_gripper()
        finally:
            raise _OperationCanceled("canceled")

    def _is_home(self):
        position_tolerance = float(self.get_parameter("home_tolerance").value)
        orientation_tolerance = float(
            self.get_parameter("home_orientation_tolerance").value
        )
        distance = np.linalg.norm(np.asarray(self.curr_pos) - HOME_POSE.position)
        current = np.asarray(self.curr_ori_wxyz, dtype=float)
        target = np.asarray(HOME_POSE.orientation_wxyz, dtype=float)
        current /= np.linalg.norm(current)
        target /= np.linalg.norm(target)
        angle = 2.0 * np.arccos(np.clip(abs(np.dot(current, target)), 0.0, 1.0))
        return distance <= position_tolerance and angle <= orientation_tolerance

    def _release_home(self):
        self.open()
        if not self._is_home():
            self.home()

    def _carry_home(self):
        if not self._is_home():
            self.home()

    def _restore_normal_stiffness(self):
        try:
            self.set_stiffness(
                self.K_pos, self.K_pos, self.K_pos,
                self.K_ori, self.K_ori, self.K_ori, 0,
            )
        except Exception as exc:
            self.get_logger().error(f"Could not restore stiffness: {exc}")

    def _start_inputs(self):
        self.keyboard_start()
        self.frankabuttons_start()
        self.joy_start()

    def _stop_inputs(self):
        for stop in (self.keyboard_stop, self.frankabuttons_stop, self.joy_stop):
            try:
                stop()
            except Exception as exc:
                self.get_logger().warning(f"Could not stop input listener: {exc}")

    # --- global cancellation coordination -------------------------------

    def _request_active_cancel(self):
        with self._state_lock:
            if self._active_goal is None or self._cancel_sent:
                return
            self._cancel_sent = True
            goal_id = self._active_goal.goal_id
            action_name = self._active_action_name

        try:
            self.stop()
            self.stop_gripper()
        except Exception as exc:
            self.get_logger().warning(f"Could not stop motion immediately: {exc}")

        request = CancelGoal.Request()
        request.goal_info.goal_id = goal_id
        try:
            future = self._cancel_clients[action_name].call_async(request)
            future.add_done_callback(
                lambda response_future: self._cancel_response(
                    response_future, bytes(goal_id.uuid)
                )
            )
        except Exception as exc:
            with self._state_lock:
                self._cancel_error = f"could not request cancellation: {exc}"

    def _cancel_response(self, future, requested_goal_id):
        error = ""
        try:
            response = future.result()
            selected = any(
                bytes(info.goal_id.uuid) == requested_goal_id
                for info in response.goals_canceling
            )
            with self._state_lock:
                active_matches = (
                    self._active_goal is not None
                    and self._goal_id(self._active_goal) == requested_goal_id
                )
                already_canceling = (
                    active_matches and self._active_goal.is_cancel_requested
                )
            if response.return_code not in (
                CancelGoal.Response.ERROR_NONE,
                CancelGoal.Response.ERROR_GOAL_TERMINATED,
            ) and not already_canceling:
                error = f"cancel request failed with code {response.return_code}"
            elif (
                response.return_code == CancelGoal.Response.ERROR_NONE
                and not selected
                and active_matches
                and not already_canceling
            ):
                error = "cancel request did not select the active operation"
        except Exception as exc:
            error = f"cancel request failed: {exc}"
        if error:
            self.get_logger().error(error)
            with self._state_lock:
                self._cancel_error = error

    @staticmethod
    def _goal_id(goal_handle):
        return bytes(goal_handle.goal_id.uuid)

    # --- progress and local GUI -----------------------------------------

    def _publish_feedback(
        self, goal_handle, phase, parts, index=0, part=None, progress=0.0
    ):
        if not goal_handle.is_active:
            return
        feedback = ExecuteSkill.Feedback()
        feedback.phase = phase
        feedback.part_index = index
        feedback.part_count = len(parts)
        feedback.skill_name = "" if part is None else part.name
        feedback.part_progress = float(max(0.0, min(1.0, progress)))
        try:
            goal_handle.publish_feedback(feedback)
        except Exception as exc:
            self.get_logger().warning(f"Could not publish task feedback: {exc}")

    def _queue_signalizer(self, state):
        self._signalizer_states.put(state)

    def pump_signalizer(self):
        latest = None
        try:
            while True:
                latest = self._signalizer_states.get_nowait()
        except queue.Empty:
            pass
        if latest == "executing":
            self.signalizer.signalize_execution()
        elif latest == "ready":
            self.signalizer.signalize_ready_demonstration()
        elif latest == "recording":
            self.signalizer.signalize_demonstration()
        elif latest == "idle":
            self.signalizer.signalize_idle()
        self.signalizer.root.update()

    @staticmethod
    def _result_message(task, outcome, completed):
        suffix = f"; completed parts: {', '.join(completed)}" if completed else ""
        return f"Task {task.command!r} {outcome}{suffix}"


def main():
    rclpy.init()
    server = None
    failed = False
    try:
        server = LfDServer()
        server.start()
        server.get_logger().info(
            "LfD server ready: execute, record, home, and template reservation"
        )
        while rclpy.ok():
            server.pump_signalizer()
            time.sleep(0.05)
        print("LfD server exiting: rclpy context is no longer ok", flush=True)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        failed = True
        print(f"LfD server failed: {exc}", flush=True)
        traceback.print_exc()
    finally:
        if server is not None:
            try:
                server._stop_inputs()
                server.signalizer.close()
                server.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()
        os._exit(1 if failed else 0)  # panda_py Desk listener is not a daemon thread


if __name__ == "__main__":
    main()
