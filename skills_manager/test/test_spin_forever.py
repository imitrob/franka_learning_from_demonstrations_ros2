"""The spin thread must survive a callback that raises."""
import types

from skills_manager import ros_utils


class _Logger:
    def __init__(self):
        self.errors = []

    def error(self, message):
        self.errors.append(message)


def test_spin_forever_restarts_after_a_callback_error(monkeypatch):
    calls = []

    def spin():
        calls.append(len(calls))
        if len(calls) < 3:
            raise RuntimeError("Failed to send goal response")

    logger = _Logger()
    node = types.SimpleNamespace(get_logger=lambda: logger)
    monkeypatch.setattr(ros_utils.rclpy, "ok", lambda: True)

    ros_utils.SpinningRosNode._spin_forever(node, types.SimpleNamespace(spin=spin))

    assert len(calls) == 3          # restarted twice, then returned cleanly
    assert len(logger.errors) == 2  # both failures logged, neither fatal
