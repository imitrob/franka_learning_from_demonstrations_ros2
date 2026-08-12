import os
import queue
import threading
from types import SimpleNamespace

import numpy as np

os.environ.setdefault("PYNPUT_BACKEND", "dummy")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/mpl-lfd-server-tests")

from rclpy.action import GoalResponse

from lfd_msgs.msg import OperationStatus
from multi_modal_reasoning.skill_command import SkillCommand
from skills_manager.lfd_server import LfDServer


def _task(action, objects=None, parameters=None):
    task = SkillCommand("", {})
    task.action = action
    task.objects = objects or []
    task.parameters = parameters or {}
    return task


class _Logger:
    def __getattr__(self, _name):
        return lambda *_args, **_kwargs: None


class _AdmissionServer(LfDServer):
    def __init__(self):
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
        self._deferred_stop_ids = set()
        self._stop_errors = {}
        self._stop_outcomes = {}
        self._cancel_sent = False
        self._cancel_error = ""
        self._recording_id = ""
        self._record_deadline = 0.0
        self._lease_id = ""
        self._lease_deadline = 0.0
        self.cancel_requests = 0

    def get_logger(self):
        return _Logger()

    def _request_active_cancel(self):
        self.cancel_requests += 1

    def get_parameter(self, _name):
        return SimpleNamespace(value=30.0)


def _request(task):
    return SimpleNamespace(skill_command_json=task.to_json())


def test_task_expands_to_one_or_two_skill_parts():
    assert [part.name for part in LfDServer.parts_for_task(
        _task("pick", ["cube"])
    )] == ["pick__cube"]
    assert [part.name for part in LfDServer.parts_for_task(
        _task("put", ["cube", "bowl"])
    )] == ["put1__cube", "put2__bowl"]


def test_goal_callback_reserves_busy_before_execution_starts():
    server = _AdmissionServer()

    assert server._execute_goal_callback(_request(_task("pick", ["cube"]))) \
        == GoalResponse.ACCEPT
    assert server._execute_goal_callback(_request(_task("pick", ["bowl"]))) \
        == GoalResponse.REJECT
    assert server._execute_goal_callback(_request(_task("stop"))) == GoalResponse.ACCEPT
    assert server._execute_goal_callback(_request(_task("pick", ["bowl"]))) \
        == GoalResponse.REJECT


def test_all_robot_actions_use_the_same_operation_gate():
    server = _AdmissionServer()
    home = SimpleNamespace(height=0.4, front_offset=0.4, side_offset=0.0)

    assert server._home_goal_callback(home) == GoalResponse.ACCEPT
    assert server._operation_mode == OperationStatus.HOMING
    assert server._reserve_goal_callback(SimpleNamespace(requester="template")) \
        == GoalResponse.REJECT


def test_recording_name_derives_the_object_and_rejects_missing_object():
    request = SimpleNamespace(skill_name="pick__cube")
    part = LfDServer._recording_part(request)
    assert (part.action, part.object) == ("pick", "cube")

    for invalid in ("", "pick", "__cube", "pick__cube_trial_1"):
        try:
            LfDServer._recording_part(SimpleNamespace(skill_name=invalid))
        except ValueError:
            pass
        else:
            raise AssertionError(f"accepted invalid recording name {invalid!r}")


def test_server_rejects_unsupported_parameters_and_arities():
    for task in (
        _task("pick", ["cube"], {"speed": "slow"}),
        _task("home"),
        _task("pick", [""]),
        _task("put", ["one", "two", "three"]),
    ):
        try:
            LfDServer._validate_supported_task(task)
        except ValueError:
            pass
        else:
            raise AssertionError(f"accepted unsupported task {task.to_dict()}")


class _Goal:
    def __init__(self, task=None, uuid=None):
        self.is_active = True
        self.is_cancel_requested = False
        self.status = None
        self.feedback = []
        self.request = None if task is None else _request(task)
        self.goal_id = SimpleNamespace(uuid=uuid or bytes(range(16)))
        self.execute_count = 0

    def execute(self):
        self.execute_count += 1

    def publish_feedback(self, feedback):
        self.feedback.append(feedback)

    def succeed(self):
        self.status = "succeeded"
        self.is_active = False

    def abort(self):
        self.status = "aborted"
        self.is_active = False

    def canceled(self):
        self.status = "canceled"
        self.is_active = False


class _ExecutionServer(LfDServer):
    def __init__(self):
        self.events = []
        self._signalizer_states = queue.SimpleQueue()

    def get_logger(self):
        return _Logger()

    def _queue_signalizer(self, state):
        self.events.append(("signal", state))

    def _start_inputs(self):
        self.events.append(("inputs", "start"))

    def _stop_inputs(self):
        self.events.append(("inputs", "stop"))

    def _set_operation_phase(self, phase, message=""):
        self.events.append(("phase", phase, message))

    def _publish_feedback(self, _goal, phase, _parts, index=0, part=None,
                          progress=0.0):
        self.events.append(("feedback", phase, index,
                            None if part is None else part.name, progress))

    def _release_home(self):
        self.events.append(("release_home",))

    def _carry_home(self):
        self.events.append(("carry_home",))

    def _validate_parts(self, _goal, parts):
        self.events.append(("validate", [part.name for part in parts]))

    def _localize_part(self, part):
        self.events.append(("localize", part.name))

    def _execute_part(self, _goal, _parts, _index, part):
        self.events.append(("execute", part.name))

    def _finish_operation(self, cleanup_error, outcome):
        self.events.append(("finish", cleanup_error, outcome))


def test_multipart_task_carries_object_between_parts_and_releases_at_boundaries():
    server = _ExecutionServer()
    goal = _Goal()

    result = server._execute_task(goal, _task("put", ["cube", "bowl"]))

    assert goal.status == "succeeded"
    assert result.completed_parts == ["put1__cube", "put2__bowl"]
    assert server.events.count(("release_home",)) == 2
    assert server.events.count(("carry_home",)) == 1
    assert ("execute", "put1__cube") in server.events
    assert ("carry_home",) in server.events
    assert ("execute", "put2__bowl") in server.events
    assert server.events[-2:] == [
        ("signal", "idle"),
        ("finish", "", "completed"),
    ]


def test_stop_waits_for_the_reserved_task_then_completes_after_cleanup():
    server = _AdmissionServer()
    task = _task("pick", ["cube"])
    stop = _task("stop")
    task_goal = _Goal(task, bytes(range(16)))
    stop_goal = _Goal(stop, bytes(range(1, 17)))

    assert server._execute_goal_callback(_request(task)) == GoalResponse.ACCEPT
    assert server._execute_goal_callback(_request(stop)) == GoalResponse.ACCEPT

    # A stop can arrive before the accepted task has its server goal handle.
    server._execute_accepted_callback(stop_goal)
    server._execute_accepted_callback(task_goal)

    assert stop_goal.execute_count == 0
    assert task_goal.execute_count == 1
    assert server.cancel_requests == 1

    server._finish_operation("", "canceled")

    assert stop_goal.execute_count == 1
    assert server._operation_mode == OperationStatus.IDLE


class _CancelExecutionServer(_ExecutionServer):
    def stop(self):
        self.events.append(("stop",))

    def stop_gripper(self):
        self.events.append(("stop_gripper",))

    def _execute_part(self, goal, _parts, _index, part):
        self.events.append(("execute", part.name))
        goal.is_cancel_requested = True
        self._raise_if_canceled(goal)


def test_cancel_stops_the_whole_task_and_still_release_homes():
    server = _CancelExecutionServer()
    goal = _Goal()

    result = server._execute_task(goal, _task("put", ["cube", "bowl"]))

    assert goal.status == "canceled"
    assert result.completed_parts == []
    assert ("execute", "put2__bowl") not in server.events
    assert ("stop",) in server.events
    assert ("stop_gripper",) in server.events
    assert server.events.count(("release_home",)) == 2
    assert server.events[-1] == ("finish", "", "canceled")


class _CleanupFailureServer(_ExecutionServer):
    def __init__(self):
        super().__init__()
        self.release_count = 0

    def _release_home(self):
        self.release_count += 1
        self.events.append(("release_home",))
        if self.release_count == 2:
            raise RuntimeError("home failed")


def test_cleanup_failure_aborts_and_is_reported_once():
    server = _CleanupFailureServer()
    goal = _Goal()

    result = server._execute_task(goal, _task("pick", ["cube"]))

    assert goal.status == "aborted"
    assert "cleanup failed: home failed" in result.message
    assert server.release_count == 2
    assert server.events[-1] == ("finish", "home failed", "failed")


class _SuccessCancelRaceGoal(_Goal):
    def succeed(self):
        self.is_cancel_requested = True
        raise RuntimeError("goal entered canceling")


def test_cancel_wins_a_race_with_the_success_transition():
    server = _ExecutionServer()
    goal = _SuccessCancelRaceGoal()

    result = server._execute_task(goal, _task("pick", ["cube"]))

    assert goal.status == "canceled"
    assert "canceled" in result.message
    assert server.events[-1] == ("finish", "", "canceled")


class _HomeServer(LfDServer):
    def __init__(self, position, orientation):
        self._position = position
        self._orientation = orientation
        self.events = []

    @property
    def curr_pos(self):
        return self._position

    @property
    def curr_ori_wxyz(self):
        return self._orientation

    def get_parameter(self, name):
        value = 0.05 if name == "home_tolerance" else 0.1
        return SimpleNamespace(value=value)

    def open(self):
        self.events.append("open")

    def home(self):
        self.events.append("home")


def test_release_home_checks_position_and_orientation():
    at_home = _HomeServer([0.4, 0.0, 0.4], [0.0, -1.0, 0.0, 0.0])
    wrong_orientation = _HomeServer(
        [0.4, 0.0, 0.4], [1.0, 0.0, 0.0, 0.0]
    )

    at_home._release_home()
    wrong_orientation._release_home()

    assert at_home.events == ["open"]
    assert wrong_orientation.events == ["open", "home"]
