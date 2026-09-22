"""Hardware-free checks: exercise the real motion loops and control handoff."""
import os
os.environ.setdefault("PYNPUT_BACKEND", "dummy")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/mpl-motion-tests")

import math
import threading
import time
from contextlib import contextmanager
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest
import quaternion

from panda_control.panda import Panda, MotionCanceled, MotionError, HIGH_ORI_DIFFERENCE
from panda_control.pose_transform_functions import min_angle_condition, pos_quat_2_pose_st
from skills_manager.lfd import LfD


class Robot(Panda):
    def __init__(self):
        self._init_motion_state()
        self._controller_ready.set()
        self.position = np.zeros(3)
        self.orientation = np.array([0., 0., 0., 1.])
        self.goal_position = self.goal_orientation = None
        self.K_pos, self.K_ori, self.K_ns = 1000, 30, 0
        self.translational_stiffness_X = self.translational_stiffness_Y = self.translational_stiffness_Z = 1000
        self.rotational_stiffness_X = self.rotational_stiffness_Y = self.rotational_stiffness_Z = 30
        self.panda = SimpleNamespace(get_state=lambda: SimpleNamespace(
            cartesian_contact=[False] * 6, cartesian_collision=[False] * 6, joint_collision=[False] * 7))
        self.attractor_distance_threshold = 0.05
        self.break_control_requested = threading.Event()
        self.break_control_done = threading.Event()
        self.tracking_timeout = 0.01
        self.commands = []
        self.follow = False
        self.ctrl = SimpleNamespace(set_control=self.control, set_impedance=lambda _: None)

    def get_logger(self):
        return SimpleNamespace(warning=lambda *_: None, info=lambda *_: None)

    def check_motion(self):
        with patch("panda_control.panda.rclpy.ok", return_value=True):
            super().check_motion()

    @property
    def curr_pos(self):
        return self.position.copy()

    @property
    def curr_ori_xyzw(self):
        return self.orientation.copy()

    @property
    def curr_ori_wxyz(self):
        return self.orientation[[3, 0, 1, 2]]

    @property
    def curr_pose(self):
        return pos_quat_2_pose_st(self.position, quaternion.quaternion(*self.curr_ori_wxyz))

    @property
    def curr_joint(self):
        return np.zeros(7)

    def create_rate(self, _):
        return SimpleNamespace(sleep=lambda: None)

    def set_stiffness(self, *_):
        self.check_motion()

    def stop_gripper(self):
        pass

    def control(self, position, orientation):
        self.commands.append((np.array(position), np.array(orientation)))
        if self.follow:
            self.position = np.array(position)
            self.orientation = np.array(orientation)

    def motion_sleep(self, seconds):
        self._control_step(self.ctrl)
        super().motion_sleep(min(seconds, 0.002))

    def wait_for_hold(self):
        self._control_step(self.ctrl)
        super().wait_for_hold()


def pose(x, angle=0.0):
    return pos_quat_2_pose_st(np.array([float(x), 0., 0.]),
                             quaternion.quaternion(math.cos(angle / 2), 0., 0., math.sin(angle / 2)))


@contextmanager
def running(robot, action):
    def work():
        with robot.robot_operation():
            return action()
    with patch("panda_control.panda.rclpy.ok", return_value=True), ThreadPoolExecutor(max_workers=1) as pool:
        future = pool.submit(work)
        try:
            yield future
        finally:
            if not future.done():
                robot.stop()  # Also keep failing tests from leaving a paused worker behind.


def wait_until(condition):
    deadline = time.monotonic() + 2
    while not condition():
        assert time.monotonic() < deadline, "worker did not reach expected state"
        time.sleep(0.005)


def ik_model(success=True):
    return SimpleNamespace(ikine_LM=lambda *_, **__: SimpleNamespace(success=success, q=np.zeros(7)))


def test_quaternion_angle_and_invalid_inputs():
    identity = [0., 0., 0., 1.]
    assert min_angle_condition(identity, [0., 0., 1., 0.]) == pytest.approx(math.pi)
    assert min_angle_condition(identity, [0., 0., 0., -2.]) == 0
    for invalid in ([0, 0, 0, 0], [0, 0, 0, np.nan], [0, 0, 1]):
        with pytest.raises(ValueError):
            min_angle_condition(identity, invalid)


@pytest.mark.parametrize("target", [pose(float("nan")), pose(float("inf"))])
def test_bad_target_holds_and_next_operation_can_run(target):
    robot = Robot()
    with pytest.raises(MotionError), robot.robot_operation():
        robot.move_to_pose_with_stampedpose(target)
    assert robot._hold_done.is_set()
    assert all(np.array_equal(p, robot.position) for p, _ in robot.commands)
    with robot.robot_operation():
        robot.move_to_pose_with_stampedpose(pose(0.01))
        robot._control_step(robot.ctrl)
    assert robot.commands[-1][0][0] == 0.01


def test_stop_cannot_be_overwritten_and_requires_acknowledgement():
    robot = Robot()
    robot.move_to_pose_with_stampedpose(pose(0.04))
    robot._control_step(robot.ctrl)
    robot.stop()
    with pytest.raises(MotionCanceled):
        robot.move_to_pose_with_stampedpose(pose(0.04))
    with pytest.raises(MotionError, match="acknowledged"):
        robot.begin_motion()
    robot._control_step(robot.ctrl)
    assert np.array_equal(robot.commands[-1][0], robot.position)
    count = len(robot.commands)
    robot._control_step(robot.ctrl)
    assert len(robot.commands) == count  # hold target stays fixed
    robot.begin_motion()


def test_controller_detects_tracking_jump_and_keeps_servicing_stop():
    robot = Robot()
    robot.move_to_pose_with_stampedpose(pose(0.04))
    robot.position[0] = -0.1
    robot._control_step(robot.ctrl)
    assert not robot._motion_cancel.is_set()
    assert robot._motion_fault == ""
    assert robot._recovery_requested.is_set() and robot._hold_done.is_set()
    assert robot.commands[-1][0][0] == -0.1
    assert "position=" in robot._recovery_reason and "orientation=" in robot._recovery_reason


@pytest.mark.parametrize("method", ["go_to_pose_ik", "go_to_pose_ik_quick"])
def test_stationary_robot_pauses_then_continues_same_interpolation(method):
    robot = Robot()
    with patch("panda_control.panda.rtb.models.Panda", return_value=ik_model()):
        with running(robot, lambda: getattr(robot, method)(pose(0.08), goal_configuration=np.zeros(7))) as future:
            assert robot._hold_done.wait(2)
            assert not future.done() and robot._operation_depth == 1
            assert robot._motion_fault == "" and not robot._motion_cancel.is_set()
            assert robot._recovery_reason == "Tracking timed out"
            robot.follow = True
            assert robot.resume_motion()
            future.result(timeout=3)
    assert robot.position[0] == pytest.approx(0.08)
    assert robot._operation_depth == 0 and robot._recovery_target is None


@pytest.mark.parametrize("method", ["go_to_pose_ik", "go_to_pose_ik_quick"])
def test_long_move_and_large_rotation_interpolate_within_guards(method):
    robot = Robot()
    robot.follow = True
    with patch("panda_control.panda.rtb.models.Panda", return_value=object()):
        with robot.robot_operation():
            getattr(robot, method)(pose(0.3, math.pi), goal_configuration=np.zeros(7))
    assert robot.position[0] == pytest.approx(0.3)
    assert min_angle_condition(robot.orientation, [0., 0., 1., 0.]) < 1e-7
    assert all(min_angle_condition(a[1], b[1]) <= HIGH_ORI_DIFFERENCE
               for a, b in zip(robot.commands, robot.commands[1:]))


def test_tracking_lag_recovers_but_republishing_does_not_reset_deadline():
    robot = Robot()
    with patch("panda_control.panda.time.monotonic", return_value=1.0):
        robot.move_to_pose_with_stampedpose(pose(0.07))
    assert not robot.safety_check
    robot.position[0] = 0.03
    assert robot.safety_checker()
    assert robot._tracking_since is None
    robot.position[0] = 0
    with patch("panda_control.panda.time.monotonic", return_value=2.0):
        robot.move_to_pose_with_stampedpose(pose(0.07))
    # Same target stays outstanding; polling must not reset its deadline.
    with patch("panda_control.panda.time.monotonic", return_value=3.0):
        robot._control_step(robot.ctrl)
    assert robot._recovery_requested.is_set()
    assert robot._recovery_reason == "Tracking timed out"
    assert not robot._motion_cancel.is_set()


def test_second_operation_rejected_while_stop_bypasses_admission():
    robot = Robot()
    entered, release = threading.Event(), threading.Event()
    def first():
        with robot.robot_operation():
            entered.set()
            assert release.wait(1)
    with ThreadPoolExecutor(max_workers=1) as pool:
        future = pool.submit(first)
        assert entered.wait(1)
        try:
            with pytest.raises(MotionError, match="already active"), robot.robot_operation():
                pass
            robot.stop()
            assert robot._motion_cancel.is_set()
        finally:
            release.set()
        future.result(timeout=1)
    robot.wait_for_hold()
    with robot.robot_operation():
        assert not robot._motion_cancel.is_set()


def test_localizer_goals_run_on_callers_thread_and_scope_is_closed():
    robot = Robot()
    robot.follow = True
    worker = threading.get_ident()
    calls = []
    robot.go_to_pose_ik_quick = lambda p: calls.append((threading.get_ident(), p))
    target = pose(0.01)
    def send(_):
        robot.external_call(target)
        return SimpleNamespace(done=lambda: True, result=lambda: "localized")
    client = SimpleNamespace(call_async=send)
    assert LfD.call_motion_service(robot, client, object(), localizing=True) == "localized"
    assert calls == [(worker, target)]
    robot.external_call(target)
    assert robot.external_call_msg is None
    assert not robot._accept_localizer_goals


@pytest.mark.parametrize("method", ["go_to_pose_ik", "go_to_pose_ik_quick"])
def test_cancel_inside_interpolation_exits_without_final_goal(method):
    robot = Robot()
    robot.follow = True
    def cancel_on_dwell(_):
        robot.stop()
        robot.check_motion()
    robot.motion_sleep = cancel_on_dwell
    with patch("panda_control.panda.rtb.models.Panda", return_value=object()):
        with pytest.raises(MotionCanceled), robot.robot_operation():
            getattr(robot, method)(pose(0.3), goal_configuration=np.zeros(7))
    assert robot.goal_position is None
    assert all(p[0] == 0 for p, _ in robot.commands)


def test_service_cancel_discards_queued_poses():
    robot = Robot()
    def send(_):
        robot.external_call(pose(0.01))
        robot.stop()
        return SimpleNamespace(done=lambda: False)
    with pytest.raises(MotionCanceled):
        LfD.call_motion_service(robot, SimpleNamespace(call_async=send), object(), localizing=True)
    assert robot.external_call_msg is None
    assert not robot._accept_localizer_goals
    assert not robot.commands


def test_playback_pause_preserves_index_samples_and_gripper_then_resumes():
    from geometry_msgs.msg import Point
    from builtin_interfaces.msg import Time
    class Playback(Robot):
        force = Point()
        grip_value = 0.0
    robot = Playback()
    robot.loaded_traj = np.array([[0.07], [0.0], [0.0]])
    robot.loaded_ori_wxyz = np.array([[1.0], [0.0], [0.0], [0.0]])
    robot.loaded_gripper = robot.loaded_img_feedback_flag = robot.loaded_spiral_flag = np.zeros((1, 1))
    robot.camera_correction = np.zeros(3)
    robot.time_index, robot.freq = 0, 10
    robot.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time))
    robot.correct = lambda: None
    gripper_calls = []
    def gripper_step(target):
        assert robot.goal_position is None  # Preserve the normal gripper-before-waypoint order.
        gripper_calls.append(target)
    robot.gripper_step = gripper_step
    robot.sift_hold_counter, robot.max_sift_hold_steps = 0, 50
    robot.sift_converged = True
    robot.recorded_traj = np.zeros((3, 1))
    robot.recorded_ori_wxyz = np.zeros((4, 1))
    robot.recorded_gripper = np.zeros((1, 1))
    robot.recorded_img = np.zeros((1, 2, 2))
    robot.pub_rec_image = lambda: np.zeros((1, 2, 2))
    robot.recorded_img_feedback_flag = robot.recorded_spiral_flag = np.zeros((1, 1))
    robot.img_feedback_flag = robot.spiral_flag = 0
    with patch("panda_control.panda.rtb.models.Panda", return_value=ik_model()):
        with running(robot, lambda: LfD.player_step(robot)) as future:
            assert robot._hold_done.wait(2)
            assert not future.done()
            assert robot.time_index == 0 and robot.recorded_traj.shape[1] == 1
            assert len(gripper_calls) == 1
            robot.follow = True
            assert robot.resume_motion()
            future.result(timeout=3)
    assert robot.time_index == 1 and robot.recorded_traj.shape[1] == 2
    assert len(gripper_calls) == 1  # Recovery does not replay the waypoint's gripper action.


@pytest.mark.parametrize("target", [pose(0.3), pose(0, math.pi), pose(0.3, math.pi)])
def test_large_divergence_holds_until_consent_then_interpolates_from_measured_pose(target):
    robot = Robot()
    with patch("panda_control.panda.rtb.models.Panda", return_value=ik_model()):
        with running(robot, lambda: robot.move_to_pose_with_stampedpose(target)) as future:
            assert robot._hold_done.wait(2)
            assert not future.done() and not robot._motion_cancel.is_set()
            with pytest.raises(MotionError, match="already active"), robot.robot_operation():
                pass
            assert all(np.array_equal(p, np.zeros(3)) for p, _ in robot.commands)
            # The operator can change the measured pose before confirming the path.
            robot.position[0] = -0.12
            robot.orientation = np.array([0., 0., math.sin(0.1), math.cos(0.1)])
            start_pos, start_ori = robot.curr_pos, robot.curr_ori_xyzw
            first = len(robot.commands)
            robot.follow = True
            assert robot.resume_motion()
            future.result(timeout=4)
    recovery = robot.commands[first:]
    assert np.allclose(recovery[0][0], start_pos)
    assert min_angle_condition(recovery[0][1], start_ori) < 1e-7
    assert all(np.linalg.norm(a[0] - b[0]) <= 0.002001 for a, b in zip(recovery, recovery[1:]))
    assert all(min_angle_condition(a[1], b[1]) <= HIGH_ORI_DIFFERENCE / 4 + 1e-7
               for a, b in zip(recovery, recovery[1:]))
    assert robot.position[0] == pytest.approx(target.pose.position.x)
    assert robot._recovery_target is None and robot._motion_fault == ""


@pytest.mark.parametrize("target", [pose(0.07), pose(0, 0.07), pose(0.07, 0.07)])
def test_small_divergence_waits_for_both_errors_without_operator_resume(target):
    robot = Robot()
    robot.tracking_timeout = 2
    entered = threading.Event()
    def move():
        robot.move_to_pose_with_stampedpose(target)
        entered.set()
        robot.motion_sleep(0.01)
    with running(robot, move) as future:
        assert entered.wait(1)
        assert not future.done() and not robot._recovery_requested.is_set()
        robot.follow = True
        robot._control_step(robot.ctrl)
        future.result(timeout=1)
    assert robot._tracking_since is None and robot._recovery_target is None


def test_unreachable_recovery_stays_paused_until_another_explicit_attempt():
    robot = Robot()
    robot.follow = True
    with patch("panda_control.panda.rtb.models.Panda", return_value=ik_model(False)) as model:
        with running(robot, lambda: robot.move_to_pose_with_stampedpose(pose(0.3))) as future:
            assert robot._hold_done.wait(2)
            saved = robot._recovery_target
            assert robot.resume_motion()
            wait_until(lambda: robot._recovery_reason == "No feasible joint configuration found"
                       and robot._hold_done.is_set())
            assert not future.done() and robot._recovery_target == saved
            assert model.call_count == 1
            time.sleep(0.03)
            assert model.call_count == 1  # No automatic IK retry loop.
            model.return_value = ik_model()
            assert robot.resume_motion()
            future.result(timeout=3)
    assert robot.position[0] == pytest.approx(0.3)


def test_obstruction_during_recovery_preserves_original_target_and_waits_again():
    robot = Robot()
    with patch("panda_control.panda.rtb.models.Panda", return_value=ik_model()):
        with running(robot, lambda: robot.move_to_pose_with_stampedpose(pose(0.2))) as future:
            assert robot._hold_done.wait(2)
            saved = robot._recovery_target
            assert robot.resume_motion()
            wait_until(lambda: robot._recovery_reason == "Tracking timed out" and robot._hold_done.is_set())
            assert not future.done() and robot._recovery_target == saved
            assert not robot._resume_requested.is_set()
            robot.follow = True
            assert robot.resume_motion()
            future.result(timeout=3)
    assert robot.position[0] == pytest.approx(0.2)


@pytest.mark.parametrize("during_recovery", [False, True])
def test_explicit_stop_cancels_instead_of_resuming(during_recovery):
    robot = Robot()
    robot.follow = True
    original_sleep = robot.motion_sleep
    def cancel_on_recovery_dwell(seconds):
        if robot._recovering:
            robot.stop()
        original_sleep(seconds)
    if during_recovery:
        robot.motion_sleep = cancel_on_recovery_dwell
    with patch("panda_control.panda.rtb.models.Panda", return_value=ik_model()):
        with running(robot, lambda: robot.move_to_pose_with_stampedpose(pose(0.2))) as future:
            assert robot._hold_done.wait(2)
            if during_recovery:
                assert robot.resume_motion()
            else:
                robot.stop()
            with pytest.raises(MotionCanceled):
                future.result(timeout=2)
    assert robot.goal_position is None and not robot.resume_motion()
    assert all(p[0] == 0 for p, _ in robot.commands)


def test_contact_escalates_small_error_and_prevents_resume_until_clear():
    robot = Robot()
    state = robot.panda.get_state()
    state.cartesian_contact[0] = True
    robot.panda.get_state = lambda: state
    robot.move_to_pose([0.07, 0., 0.], [0., 0., 0., 1.], 0.2)
    robot._control_step(robot.ctrl)
    assert robot._recovery_reason == "Contact while tracking" and robot._hold_done.is_set()
    assert robot.commands[-1][0][0] == 0
    assert not robot.resume_motion()
    state.cartesian_contact[0] = False
    assert robot.resume_motion()


@pytest.mark.parametrize("recovering", [False, True])
@pytest.mark.parametrize("method,args", [
    ("move_gripper", (0.06,)), ("grasp_gripper", (0.0,)),
    ("move", (0.06,)), ("grasp", (0.0,)), ("home_gripper", ()),
])
def test_all_gripper_entry_points_reject_recovery(method, args, recovering):
    robot = Robot()
    calls = []
    robot.gripper = SimpleNamespace(**{
        name: lambda *a, **kw: calls.append((a, kw))
        for name in ("move", "grasp", "homing", "stop")
    })
    robot._request_recovery("test", [0.2, 0., 0.], [0., 0., 0., 1.])
    robot._control_step(robot.ctrl)
    if recovering:
        robot._recovery_requested.clear()
        robot._recovering = True
    with pytest.raises(MotionError, match="recovery"):
        getattr(robot, method)(*args)
    assert calls == []
    robot._recovering = False
    robot.begin_motion()
    getattr(robot, method)(*args)
    assert calls  # Normal commands still work after recovery ends.


def test_resume_requires_controller_hold_and_no_controller_fault():
    robot = Robot()
    robot._request_recovery("test", [0.2, 0., 0.], [0., 0., 0., 1.])
    assert not robot.resume_motion()
    robot._control_step(robot.ctrl)
    robot._controller_ready.clear()
    assert not robot.resume_motion()
    robot._controller_ready.set()
    robot._motion_fault = "Controller failed"
    assert not robot.resume_motion()
    with pytest.raises(MotionError, match="Controller failed"):
        robot._recover_motion()


def test_localization_deadline_excludes_recovery_and_discards_new_poses_while_paused():
    robot = Robot()
    clock = [0.0]
    polls = []
    def done():
        polls.append(True)
        return len(polls) > 1
    def send(_):
        robot.external_call(pose(0.2))
        return SimpleNamespace(done=done, result=lambda: "localized")
    # Exercise the service wait with a correction that requires explicit recovery.
    robot.go_to_pose_ik_quick = robot.move_to_pose_with_stampedpose
    with patch("panda_control.panda.rtb.models.Panda", return_value=ik_model()), \
            patch("panda_control.panda.time.monotonic", side_effect=lambda: clock[0]):
        with running(robot, lambda: LfD.call_motion_service(
                robot, SimpleNamespace(call_async=send), object(), localizing=True, timeout=1)) as future:
            assert robot._hold_done.wait(2)
            robot.external_call(pose(0.4))
            assert robot.external_call_msg is None
            clock[0] = 100.0
            robot.follow = True
            assert robot.resume_motion()
            assert future.result(timeout=3) == "localized"
    assert robot._recovery_elapsed == 100.0
    assert robot.position[0] == pytest.approx(0.2)


def test_kinesthetic_recording_does_not_correct_or_reject_manual_roll():
    from builtin_interfaces.msg import Time
    class Recording(Robot):
        pass
    robot = Recording()
    robot.orientation = np.array([math.sin(math.radians(80)), 0., 0., math.cos(math.radians(80))])
    orientation = robot.orientation.copy()  # 20 degrees away from the preferred roll.
    robot.end = robot.pause = False
    robot.gesture_feedback = robot.joystick_feedback = None
    robot.rot_feedback, robot.feedback_gripper = [0., 0.], ""
    robot.gripper_state = SimpleNamespace(width=0.06)
    robot.img_feedback_flag = robot.spiral_flag = 0
    robot.freq = 10
    robot.r = SimpleNamespace(sleep=lambda: robot.position.__setitem__(0, 0.006))
    robot.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time))
    robot.init_additional_flags = robot.update_additional_flags = lambda: None
    robot.is_applied_external_feedback = lambda: False
    images = []
    def image():
        images.append(True)
        robot.end = len(images) == 2
        return np.zeros((1, 2, 2))
    robot.pub_rec_image = image
    assert LfD.traj_rec(robot, signalize=False, on_phase=lambda _: None)
    assert np.allclose(robot.goal_orientation, orientation)
    assert robot._recovery_target is None and not robot._teaching


def test_controller_follows_hand_guidance_without_requesting_recovery():
    robot = Robot()
    robot.move_to_pose_with_stampedpose(pose(0.0))
    robot._teaching = True
    robot.position[0] = 0.3
    robot.orientation = np.array([0., 0., 1., 0.])
    robot._control_step(robot.ctrl)
    assert robot._recovery_target is None
    assert np.allclose(robot.commands[-1][0], robot.position)
    assert np.allclose(robot.commands[-1][1], robot.orientation)
