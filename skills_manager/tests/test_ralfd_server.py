import os

os.environ.setdefault("PYNPUT_BACKEND", "dummy")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/mpl-lfd-server-tests")

import numpy as np

from skills_manager.risk_aware_lfd.ralfd_server import RALfDServer


class _Goal:
    is_cancel_requested = False


class _BranchingServer(RALfDServer):
    """Replays stub trajectories; `decisions` scripts the switcher per branch."""

    curr_pos = np.zeros(3)

    def __init__(self, decisions, lengths, end_at=None):
        self._branching = True
        self.decisions, self.lengths, self.end_at = decisions, lengths, end_at
        self.last_target_state = float("inf")  # switcher already publishing
        self.target_state = ""
        self.freq = 10
        self.played, self.saved, self.recorded = [], [], []

    def load(self, name):
        self.filename = name
        self.loaded_traj = np.zeros((3, self.lengths[name]))

    def player_init(self):
        self.time_index, self.end, self.pause = 0, False, False
        self.recorded_img = np.zeros((1, 2, 2))

    def player_step(self):
        self.played.append((self.filename, self.time_index))
        self.time_index += 1
        self.recorded_img = np.zeros((self.time_index + 1, 2, 2))
        self.target_state = self.decisions.get((self.filename, self.time_index), self.filename)
        if (self.filename, self.time_index) == self.end_at:
            self.end = 1  # the "e" key

    def _raise_if_canceled(self, _goal):
        pass

    def _feedback(self, *_args):
        pass

    def _save_trial(self, name, split=slice(None)):
        self.saved.append((name, len(self.recorded_img[split])))
        return None

    def _record_branch(self, _goal, _parts, _index, name):
        self.recorded.append(name)
        return True


def test_switcher_decision_loads_the_branch_and_saves_both_trials():
    root, branch = "pick__cube", "pick__cube_branch_from_0_at_30"
    server = _BranchingServer({(root, 12): branch}, {root: 40, branch: 5})

    server._play_branches(_Goal(), [], 1, root)

    assert server.played[-1] == (branch, 4)
    # The root trial drops the decision-state window; the finished branch keeps all.
    assert server.saved == [(root, 13 - 10), (branch, 6)]
    assert server.recorded == []


def test_end_key_records_a_new_branch_from_the_current_sample():
    root = "pick__cube"
    server = _BranchingServer({}, {root: 40}, end_at=(root, 7))

    server._play_branches(_Goal(), [], 1, root)

    assert server.recorded == ["pick__cube_branch_from_0_at_7"]
