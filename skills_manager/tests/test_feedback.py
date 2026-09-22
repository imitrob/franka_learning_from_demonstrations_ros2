import os
import threading
from types import SimpleNamespace
from unittest.mock import patch

os.environ.setdefault("PYNPUT_BACKEND", "dummy")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/mpl-lfd-feedback-tests")

from skills_manager.feedback import Feedback, FrankaOnPress
from pynput.keyboard import KeyCode


class _FeedbackServer(Feedback):
    def __init__(self):
        self.end = False
        self.gripper_state = SimpleNamespace(is_grasped=False)
        self.grip_open_width = 0.08
        self.gripper_commands = []

    def grasp_gripper(self, width):
        self.gripper_commands.append(("close", width))

    def move_gripper(self, width):
        self.gripper_commands.append(("open", width))


def test_franka_buttons_route_close_and_open_to_the_server_gripper():
    server = _FeedbackServer()

    server.franka_on_press("cross")
    server.franka_on_press("check")

    assert server.gripper_commands == [("close", 0), ("open", 0.08)]


def test_franka_finish_button_stops_recording_immediately():
    server = _FeedbackServer()

    server.franka_on_press("circle")

    assert server.end == 1


class _Desk:
    def __init__(self):
        self._listen_thread = None
        self.recv_timeout = None

    def listen(self, callback):
        self._listen_thread = threading.Thread(
            target=self._listen, args=(callback, 1.0)
        )
        self._listen_thread.start()

    def _listen(self, _callback, timeout):
        self.recv_timeout = timeout


class _FrankaButtons(FrankaOnPress):
    def __init__(self):
        self.frankabuttons_running = False
        self.desk = _Desk()


def test_franka_button_listener_has_low_shutdown_latency():
    buttons = _FrankaButtons()

    buttons.frankabuttons_start()
    buttons.desk._listen_thread.join()

    assert buttons.desk.recv_timeout <= 0.1


def test_space_confirms_recovery_without_toggling_pause_or_starting_other_motion():
    server = _FeedbackServer()
    server.pause = False
    server._recovery_target = "saved waypoint"
    resumes = []
    server.resume_motion = lambda: resumes.append(True)
    # pynput's dummy backend aliases every special key to the same enum value.
    with patch("skills_manager.feedback.Key", SimpleNamespace(esc=object(), space=object())) as keys:
        server.keyboard_on_press(keys.space)
        assert resumes == [True] and not server.pause
        for char in "comn":
            server.keyboard_on_press(KeyCode.from_char(char))
        assert server.gripper_commands == []
        server._recovery_target = None
        server.keyboard_on_press(keys.space)
        assert server.pause  # Ordinary recording pause still works.
