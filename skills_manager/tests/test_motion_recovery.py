"""Hardware-free checks: exercise the real motion loops and control handoff."""
import os
os.environ.setdefault("PYNPUT_BACKEND", "dummy")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/mpl-motion-tests")

import math
import threading
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
        self.attractor_distance_threshold = 0.05
        self.break_control_requested = threading.Event()
        self.break_control_done = threading.Event()
        self.tracking_timeout = 0.01
        self.commands = []
        self.follow = False
        self.ctrl = SimpleNamespace(set_control=self.control, set_impedance=lambda _: None)

    def get_logger(self):
        return SimpleNamespace(warning=lambda *_: None)

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


def test_quaternion_angle_and_invalid_inputs():
    identity = [0., 0., 0., 1.]
    assert min_angle_condition(identity, [0., 0., 1., 0.]) == pytest.approx(math.pi)
    assert min_angle_condition(identity, [0., 0., 0., -2.]) == 0
    for invalid in ([0, 0, 0, 0], [0, 0, 0, np.nan], [0, 0, 1]):
        with pytest.raises(ValueError):
            min_angle_condition(identity, invalid)


@pytest.mark.parametrize("target", [pose(0.11), pose(0, 0.11), pose(float("nan"))])
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
    assert robot._motion_cancel.is_set()
    assert robot._hold_done.is_set()
    assert robot.commands[-1][0][0] == -0.1
    assert "position=" in robot._motion_fault and "orientation=" in robot._motion_fault


@pytest.mark.parametrize("method", ["go_to_pose_ik", "go_to_pose_ik_quick"])
def test_stationary_robot_times_out_in_real_interpolation_loop(method):
    robot = Robot()
    with patch("panda_control.panda.rtb.models.Panda", return_value=object()):
        with pytest.raises(MotionError, match="timed out"), robot.robot_operation():
            getattr(robot, method)(pose(0.08), goal_configuration=np.zeros(7))
    assert robot._hold_done.is_set()
    assert robot._operation_depth == 0


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
    with patch("panda_control.panda.time.monotonic", return_value=3.0):
        with pytest.raises(MotionError, match="timed out"):
            robot.move_to_pose_with_stampedpose(pose(0.07))


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


def test_playback_stall_times_out_and_does_not_advance_or_grow_recording_forever():
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
    robot.gripper_step = lambda _: None
    robot.sift_hold_counter, robot.max_sift_hold_steps = 0, 50
    robot.sift_converged = True
    robot.recorded_traj = np.zeros((3, 1))
    robot.recorded_ori_wxyz = np.zeros((4, 1))
    robot.recorded_gripper = np.zeros((1, 1))
    robot.recorded_img = np.zeros((1, 2, 2))
    robot.pub_rec_image = lambda: np.zeros((1, 2, 2))
    robot.recorded_img_feedback_flag = robot.recorded_spiral_flag = np.zeros((1, 1))
    robot.img_feedback_flag = robot.spiral_flag = 0
    with patch("panda_control.panda.time.monotonic", return_value=1.0):
        LfD.player_step(robot)
    assert robot.time_index == 0
    assert robot.recorded_traj.shape[1] == 2
    with patch("panda_control.panda.time.monotonic", return_value=2.0):
        with pytest.raises(MotionError, match="timed out"):
            LfD.player_step(robot)
    assert robot.time_index == 0
    assert robot.recorded_traj.shape[1] == 2
