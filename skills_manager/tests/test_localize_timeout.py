import os

os.environ.setdefault("PYNPUT_BACKEND", "dummy")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/mpl-lfd-server-tests")

from concurrent.futures import Future
from types import SimpleNamespace

import pytest

from skills_manager.lfd import LfD


def test_hung_localizer_times_out_then_blocks_until_its_grace_ends():
    removed = []
    client = SimpleNamespace(call_async=lambda _req: Future(), remove_pending_request=removed.append)
    robot = SimpleNamespace(active_localizer_client=client, _localization_future=None,
                            _localization_deadline=0.0)
    call = lambda: LfD._call_active_localizer(robot, timeout=0.05)

    assert call() is None  # never answered: treated as "no response"
    hung = robot._localization_future
    with pytest.raises(RuntimeError, match="still running"):
        call()
    robot._localization_deadline = 0.0  # grace over
    assert call() is None and removed == [hung]


def test_ralfd_refuses_a_second_robot_operation_from_another_thread():
    import threading
    from skills_manager.risk_aware_lfd.ralfd import robot_operation

    class Robot:
        _operation_lock = threading.RLock()

        @robot_operation
        def home(self):
            return "homed"

        @robot_operation
        def play(self):
            return self.home()  # nesting in the same thread is allowed

    robot = Robot()
    assert robot.play() == "homed"
    robot._operation_lock.acquire()  # another operation holds it...
    result = []
    worker = threading.Thread(target=lambda: result.append(pytest.raises(RuntimeError, robot.home)))
    worker.start(); worker.join()
    robot._operation_lock.release()
    assert result
