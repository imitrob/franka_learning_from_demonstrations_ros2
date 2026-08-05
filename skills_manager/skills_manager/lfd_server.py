#!/usr/bin/env python3
"""Persistent ROS action server for executing HRI SkillCommand tasks."""
import queue
import threading
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from lfd_msgs.action import ExecuteSkill
from lfd_msgs.srv import SetTemplate
from multi_modal_reasoning.skill_command import SkillCommand
from panda_control.home_pose import HOME_POSE
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from std_msgs.msg import String
from trajectory_data.skill_part import SkillPart

from skills_manager.lfd import LfD


ACTION_NAME = "/lfd/execute_skill"
SKILL_COMMAND_TOPIC = "/hri/skill_command"
STOP_ACTION = "stop"
HOME_TOLERANCE = 0.05
HOME_ORIENTATION_TOLERANCE = 0.1


class _TaskCanceled(Exception):
    pass


class LfDServer(LfD):
    """Own one Panda and execute at most one multi-part LfD task at a time."""

    def __init__(self):
        super().__init__()

        self._state_lock = threading.Lock()
        self._task_busy = False
        self._active_goal = None
        self._stop_inflight = 0
        self._deferred_stop_goals = []
        self._stop_errors = {}
        self._stop_outcomes = {}
        self._deferred_stop_ids = set()
        self._cancel_sent = False
        self._cancel_error = ""

        # Tk widgets were created on the main thread by LfD. Executor workers
        # enqueue state changes; main() applies them on Tk's owning thread.
        self._signalizer_states = queue.SimpleQueue()

        self.declare_parameter("home_tolerance", HOME_TOLERANCE)
        self.declare_parameter(
            "home_orientation_tolerance", HOME_ORIENTATION_TOLERANCE
        )

        self._action_server = ActionServer(
            self,
            ExecuteSkill,
            ACTION_NAME,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            handle_accepted_callback=self._handle_accepted_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )
        self._action_client = ActionClient(
            self, ExecuteSkill, ACTION_NAME, callback_group=self.callback_group
        )
        self._cancel_client = self.create_client(
            CancelGoal,
            f"{ACTION_NAME}/_action/cancel_goal",
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            SKILL_COMMAND_TOPIC,
            self._skill_command_callback,
            10,
            callback_group=self.callback_group,
        )

    # --- admission and ROS transport -------------------------------------

    def _goal_callback(self, request):
        try:
            task = self._task_from_request(request)
            self._validate_supported_task(task)
        except ValueError as exc:
            self.get_logger().warning(f"Rejecting skill task: {exc}")
            return GoalResponse.REJECT

        with self._state_lock:
            if task.action == STOP_ACTION:
                self._stop_inflight += 1
                return GoalResponse.ACCEPT
            if self._task_busy or self._stop_inflight:
                self.get_logger().warning(
                    f"Rejecting {task.command!r}: another task or stop is active"
                )
                return GoalResponse.REJECT

            # Reserve before returning. Otherwise two goal callbacks can both
            # accept before either handle reaches _handle_accepted_callback.
            self._task_busy = True
            self._active_goal = None
            self._cancel_sent = False
            self._cancel_error = ""
        return GoalResponse.ACCEPT

    def _handle_accepted_callback(self, goal_handle):
        task = self._task_from_request(goal_handle.request)
        if task.action == STOP_ACTION:
            execute_now = False
            request_cancel = False
            with self._state_lock:
                if self._task_busy:
                    self._deferred_stop_goals.append(goal_handle)
                    self._deferred_stop_ids.add(self._goal_id(goal_handle))
                    request_cancel = self._active_goal is not None
                else:
                    execute_now = True
            if request_cancel:
                self._request_active_cancel()
            if execute_now:
                goal_handle.execute()
            return

        with self._state_lock:
            self._active_goal = goal_handle
            stop_is_waiting = self._stop_inflight > 0
        goal_handle.execute()
        if stop_is_waiting:
            self._request_active_cancel()

    def _cancel_callback(self, goal_handle):
        try:
            task = self._task_from_request(goal_handle.request)
        except ValueError:
            return CancelResponse.REJECT
        if task.action == STOP_ACTION:
            return CancelResponse.REJECT

        with self._state_lock:
            if not self._task_busy:
                return CancelResponse.REJECT
            if self._active_goal is None:
                self._active_goal = goal_handle
            elif self._active_goal is not goal_handle:
                return CancelResponse.REJECT
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        task = self._task_from_request(goal_handle.request)
        if task.action == STOP_ACTION:
            return self._execute_stop(goal_handle)
        return self._execute_task(goal_handle, task)

    def _skill_command_callback(self, message: String):
        goal = ExecuteSkill.Goal()
        goal.skill_command_json = message.data
        try:
            future = self._action_client.send_goal_async(goal)
            future.add_done_callback(self._topic_goal_response)
        except Exception as exc:  # action transport can fail during shutdown
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
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(wrapped.result.message)
        elif wrapped.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warning(wrapped.result.message)
        else:
            self.get_logger().error(wrapped.result.message)

    # --- task execution ---------------------------------------------------

    def _execute_task(self, goal_handle, task: SkillCommand):
        parts = self.parts_for_task(task)
        completed = []
        task_error = ""
        canceled = False
        cleanup_error = ""
        terminal_outcome = "failed"

        self._queue_signalizer("executing")
        try:
            self._publish_feedback(goal_handle, "homing", parts)
            self._release_home()
            self._raise_if_canceled(goal_handle)

            self._validate_parts(goal_handle, parts)
            self._raise_if_canceled(goal_handle)

            for index, part in enumerate(parts, start=1):
                if index > 1:
                    self._publish_feedback(
                        goal_handle, "homing", parts, index=index, part=part
                    )
                    self._carry_home()
                    self._raise_if_canceled(goal_handle)

                self._publish_feedback(
                    goal_handle, "localizing", parts, index=index, part=part
                )
                self._localize_part(part)
                self._raise_if_canceled(goal_handle)

                self._execute_part(goal_handle, parts, index, part)
                completed.append(part.name)
                self._raise_if_canceled(goal_handle)
        except _TaskCanceled as exc:
            canceled = True
            task_error = str(exc)
        except Exception as exc:  # all accepted tasks finish through release-home
            task_error = str(exc)
            self.get_logger().error(f"Skill task failed: {exc}")
        finally:
            try:
                self._publish_feedback(goal_handle, "homing", parts)
                self._release_home()
            except Exception as exc:
                cleanup_error = str(exc)
                self.get_logger().error(f"Release-home failed: {exc}")

            canceled = canceled or goal_handle.is_cancel_requested
            self._publish_feedback(
                goal_handle, "idle", parts, progress=1.0 if not task_error else 0.0
            )
            self._queue_signalizer("idle")

        result = ExecuteSkill.Result()
        result.completed_parts = completed
        try:
            if cleanup_error:
                result.message = self._result_message(
                    task, f"cleanup failed: {cleanup_error}", completed
                )
                goal_handle.abort()
            elif canceled or goal_handle.is_cancel_requested:
                result.message = self._result_message(
                    task, task_error or "canceled", completed
                )
                goal_handle.canceled()
                terminal_outcome = "canceled"
            elif task_error:
                result.message = self._result_message(task, task_error, completed)
                goal_handle.abort()
            else:
                result.message = self._result_message(task, "completed", completed)
                try:
                    goal_handle.succeed()
                    terminal_outcome = "completed"
                except Exception:
                    # A cancel can be accepted after the check above but before
                    # the success transition. Complete that legal transition
                    # as canceled instead of leaking an action state error.
                    if not goal_handle.is_cancel_requested:
                        raise
                    result.message = self._result_message(task, "canceled", completed)
                    goal_handle.canceled()
                    terminal_outcome = "canceled"
        finally:
            self._finish_active_task(cleanup_error, terminal_outcome)
        return result

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
                if not waited:
                    result.message = "No active task; stop is a no-op"
                elif outcome == "canceled":
                    result.message = "Active task canceled and robot homed"
                else:
                    result.message = (
                        f"Active task {outcome or 'ended'} before cancellation; "
                        "robot homed"
                    )
                goal_handle.succeed()
        finally:
            with self._state_lock:
                self._stop_inflight -= 1
        return result

    def _validate_parts(self, goal_handle, parts):
        self._publish_feedback(goal_handle, "validating", parts)
        if not self.set_localizer_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("set_localizer service is unavailable")

        for index, part in enumerate(parts, start=1):
            self._raise_if_canceled(goal_handle)
            self._publish_feedback(
                goal_handle, "validating", parts, index=index, part=part
            )
            part.validate_archive()
            response = self.set_localizer_client.call(
                SetTemplate.Request(template_name=part.object)
            )
            if response is None or not response.success:
                raise ValueError(f"localization template {part.object!r} is unavailable")

    def _localize_part(self, part: SkillPart):
        if self.localize(part.object) is False:
            raise RuntimeError(f"localization failed for {part.object!r}")

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

    # --- task policy helpers ---------------------------------------------

    @staticmethod
    def parts_for_task(task: SkillCommand) -> list[SkillPart]:
        if len(task.objects) == 1:
            return [SkillPart(f"{task.action}{SkillPart.SEP}{task.objects[0]}")]
        return [
            SkillPart(f"{task.action}{index}{SkillPart.SEP}{obj}")
            for index, obj in enumerate(task.objects, start=1)
        ]

    @staticmethod
    def _validate_supported_task(task: SkillCommand):
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
    def _task_from_request(request) -> SkillCommand:
        return SkillCommand.from_json(request.skill_command_json)

    def _raise_if_canceled(self, goal_handle):
        if not goal_handle.is_cancel_requested:
            return
        try:
            self.stop()
            self.stop_gripper()
        finally:
            raise _TaskCanceled("canceled")

    def _is_home(self) -> bool:
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
        return (
            float(distance) <= position_tolerance
            and float(angle) <= orientation_tolerance
        )

    def _release_home(self):
        self.open()
        if not self._is_home():
            self.home()

    def _carry_home(self):
        if not self._is_home():
            self.home()

    # --- cancellation coordination --------------------------------------

    def _request_active_cancel(self):
        with self._state_lock:
            if self._active_goal is None or self._cancel_sent:
                return
            self._cancel_sent = True
            goal_id = self._active_goal.goal_id

        request = CancelGoal.Request()
        request.goal_info.goal_id = goal_id
        try:
            future = self._cancel_client.call_async(request)
            future.add_done_callback(
                lambda response_future: self._cancel_response(
                    response_future, bytes(goal_id.uuid)
                )
            )
        except Exception as exc:
            with self._state_lock:
                self._cancel_error = f"could not request cancellation: {exc}"

    def _cancel_response(self, future, requested_goal_id: bytes):
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
                error = "cancel request did not select the active task"
        except Exception as exc:
            error = f"cancel request failed: {exc}"
        if error:
            self.get_logger().error(error)
            with self._state_lock:
                self._cancel_error = error

    def _finish_active_task(self, cleanup_error: str, outcome: str):
        with self._state_lock:
            stop_error = cleanup_error or self._cancel_error
            deferred = list(self._deferred_stop_goals)
            self._deferred_stop_goals.clear()
            for stop_goal in deferred:
                stop_id = self._goal_id(stop_goal)
                self._stop_errors[stop_id] = stop_error
                self._stop_outcomes[stop_id] = outcome
            self._active_goal = None
            self._task_busy = False
            self._cancel_sent = False
            self._cancel_error = ""

        for stop_goal in deferred:
            stop_goal.execute()

    @staticmethod
    def _goal_id(goal_handle) -> bytes:
        return bytes(goal_handle.goal_id.uuid)

    # --- progress and local GUI ------------------------------------------

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

    def _queue_signalizer(self, state: str):
        self._signalizer_states.put(state)

    def pump_signalizer(self):
        """Apply queued Tk changes from the main thread and process redraws."""
        latest = None
        try:
            while True:
                latest = self._signalizer_states.get_nowait()
        except queue.Empty:
            pass
        if latest == "executing":
            self.signalizer.signalize_execution()
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
    try:
        server = LfDServer()
        server.start()
        server.get_logger().info(
            f"LfD server ready: action {ACTION_NAME}, topic {SKILL_COMMAND_TOPIC}"
        )
        # SpinningRosNode already owns a MultiThreadedExecutor. The main thread
        # stays available for Tk, whose widgets may only be touched here.
        while rclpy.ok():
            server.pump_signalizer()
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"LfD server failed: {exc}", flush=True)
        raise
    finally:
        if server is not None:
            try:
                server.signalizer.close()
                server.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
