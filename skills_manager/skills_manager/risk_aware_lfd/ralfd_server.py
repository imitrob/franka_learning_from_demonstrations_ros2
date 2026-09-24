#!/usr/bin/env python3
"""LfD server whose replay branches on the switcher's decisions.

Without a running switcher (``/state_decider_retrain``), or for a goal with
``plain_replay``, every part replays exactly as in ``LfDServer``. With it, a part
follows ``/target_state`` into recorded branches, saves each run as a trial, and
the "e" key records a new branch from where the robot is.

Port of the branching loop in ``RALfD.play_skill``/``RALfD.execute``, kept
independent of the notebook class.
"""
import os
import queue
import time

import numpy as np
from lfd_msgs.action import ExecuteSkill
from lfd_msgs.srv import StringService
from nocode_robot_programming.state_decision.utils import Filename
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from trajectory_data.skill_part import SkillPart

from skills_manager import lfd_server
from skills_manager.lfd_server import LfDServer

DECISION_WINDOW = 10  # samples before a switch; not saved under the old label
TARGET_STATE_TIMEOUT = 0.15  # s; older switcher output means it is not running yet
MANUAL_MOVE = 0.02  # m; moving a paused robot this far by hand resumes it
RETRAIN_TIMEOUT = 90.0  # s; the switcher gives up after 60 s


class RALfDServer(LfDServer):
    def __init__(self):
        self._branching = False
        self._choice_requests = queue.SimpleQueue()
        self.target_state = ""
        self.last_target_state = 0.0
        super().__init__()
        self.declare_parameter("topic_plain_replay", False)
        self.create_subscription(
            String, "/target_state", self._target_state_callback, 5,
            callback_group=self.callback_group,
        )
        # The switcher serves this with best-effort QoS; a reliable client never gets the reply.
        self.retrain_client = self.create_client(
            StringService, "/state_decider_retrain",
            qos_profile=QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT),
            callback_group=self.callback_group,
        )

    def _target_state_callback(self, message):
        self.last_target_state = time.time()
        self.target_state = message.data

    def _skill_command_callback(self, message):
        goal = ExecuteSkill.Goal()
        goal.skill_command_json = message.data
        goal.plain_replay = bool(self.get_parameter("topic_plain_replay").value)
        try:
            future = self._execute_client.send_goal_async(goal)
            future.add_done_callback(self._topic_goal_response)
        except Exception as exc:
            self.get_logger().error(f"Could not forward skill command: {exc}")

    def _execute_part(self, goal_handle, parts, index, part):
        if goal_handle.request.plain_replay or not self.retrain_client.service_is_ready():
            return super()._execute_part(goal_handle, parts, index, part)
        self._set_operation_phase("training", part.name)
        self._publish_feedback(goal_handle, "training", parts, index=index, part=part)
        future = self.retrain_client.call_async(StringService.Request(text=part.name))
        deadline = time.monotonic() + RETRAIN_TIMEOUT
        while not future.done():
            self._raise_if_canceled(goal_handle)
            if time.monotonic() > deadline:
                raise RuntimeError("Switcher retrain timed out")
            time.sleep(0.05)
        self._set_operation_phase("executing", part.name)
        self._branching = True
        try:
            self._play_branches(goal_handle, parts, index, part.name)
        finally:
            self._branching = False

    def _play_branches(self, goal_handle, parts, index, name):
        saved = []
        while True:
            self.load(name)
            action, next_name = self._run_branch(goal_handle, parts, index)
            if action == "done":
                saved.append(self._save_trial(name))
                return
            saved.append(self._save_trial(name, slice(None, -DECISION_WINDOW)))
            if action == "play":
                name = next_name
                continue
            if not self._record_branch(goal_handle, parts, index, next_name):
                for trial in filter(None, saved):  # RALfD: no branch, no trials of this run
                    os.remove(SkillPart(trial).archive_path())
            return

    def _run_branch(self, goal_handle, parts, index):
        """Replay the loaded branch until it ends or the switcher leaves it.

        Returns ("done", ""), ("play", <branch to load>) or ("rec", <new branch name>).
        """
        self.player_init()
        while time.time() - self.last_target_state > TARGET_STATE_TIMEOUT and not self.end:
            self._raise_if_canceled(goal_handle)
            self._feedback(goal_handle, "waiting_for_switcher", parts, index)
            self.pub_rec_image()  # the switcher predicts only on fresh images
            time.sleep(0.2)
        while self.time_index < self.loaded_trajectory_len and not self.end:
            start = np.copy(self.curr_pos)
            while self.pause and not self.end:
                self._raise_if_canceled(goal_handle)
                time.sleep(1.0 / self.freq)
                if np.linalg.norm(self.curr_pos - start) > MANUAL_MOVE:
                    self.pause = False
            if self.end:
                break
            self._raise_if_canceled(goal_handle)
            self.player_step()
            target = self._resolve_target(goal_handle, self.target_state)
            # "" means the switcher has no model for this task yet: pause like an anomaly.
            anomaly = target in ("anomaly", "")
            self._feedback(goal_handle, "anomaly" if anomaly else "executing", parts, index)
            if anomaly:
                self.pause = True
            elif target != self.filename:
                return "play", target
        if self.time_index < self.loaded_trajectory_len:
            current = Filename(self.filename)
            branch = Filename(current.task, offset=self.time_index + current.offset,
                              parent_offset=current.offset)
            branch.find_unique()
            return "rec", branch.to_str()
        return "done", ""

    def _resolve_target(self, goal_handle, target):
        if target.startswith("manual_choose|"):
            target = self._ask_operator(goal_handle, target.split("|")[1:]) or "continue"
        return self.filename if target == "continue" else target

    def _ask_operator(self, goal_handle, options):
        reply = queue.SimpleQueue()
        self._choice_requests.put((options, reply))
        while True:
            self._raise_if_canceled(goal_handle)
            try:
                return reply.get(timeout=0.1)
            except queue.Empty:
                pass

    def pump_signalizer(self):
        super().pump_signalizer()
        try:
            options, reply = self._choice_requests.get_nowait()
        except queue.Empty:
            return
        choice = None
        try:  # Tk belongs to main(); executor workers only enqueue the question.
            from nocode_robot_programming.teaching.user_study_widget import choose_with_popup
            choice = choose_with_popup(options, title="Select target skill part!",
                                       master=self.signalizer.root)
        except Exception as exc:
            self.get_logger().error(f"Could not ask for the target skill part: {exc}")
        finally:
            reply.put(choice)

    def _record_branch(self, goal_handle, parts, index, name):
        self._branching = False  # "e" now finishes the recording, as in RecordSkill
        self.end = False
        self.pause = False
        self._set_operation_phase("recording", name)
        self._publish_feedback(goal_handle, "recording", parts, index=index, part=SkillPart(name))
        started = self.traj_rec(
            should_stop=lambda: goal_handle.is_cancel_requested,
            on_phase=lambda phase: self._queue_signalizer(
                "ready" if phase == "ready" else "recording"),
            signalize=False,
        )
        self._raise_if_canceled(goal_handle)
        self._queue_signalizer("executing")
        return bool(started) and self.save(name)

    def _save_trial(self, name, split=slice(None)):
        if len(self.recorded_img[split]) < 3:  # also covers the 1-D arrays before the first step
            return None
        for key in ("recorded_traj", "recorded_ori_wxyz", "recorded_gripper",
                    "recorded_img_feedback_flag", "recorded_spiral_flag"):
            setattr(self, key, getattr(self, key)[:, split])
        self.recorded_img = self.recorded_img[split]
        trial = Filename(name, init_exec_trial=True).to_str()
        self.save(trial)
        return trial

    def _feedback(self, goal_handle, phase, parts, index):
        self._publish_feedback(goal_handle, phase, parts, index=index,
                               part=SkillPart(self.filename), progress=self.time_phase)


def main():
    lfd_server.main(RALfDServer)


if __name__ == "__main__":
    main()
