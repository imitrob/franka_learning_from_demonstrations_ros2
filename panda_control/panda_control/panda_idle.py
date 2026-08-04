#!/usr/bin/env python3
"""Read-only pose feed for the arm: panda_link0 -> panda_hand TF and /panda/curr_pose.

Why this exists: object_localization's get_scene un-projects a matched pixel using
the *camera* pose, and the camera rides on the end effector. panda.py is what
broadcasts panda_link0 -> panda_hand, but it is only launched to record or play a
skill. So while you are merely watching /scene -- checking that saved templates
are recognised where the objects actually are -- there is no arm transform, and
get_scene correctly refuses to guess:

    [scene] TF to the camera not available, skipping

which reaches the dashboard as a permanently empty scene. This node supplies just
that transform, and nothing else.

It also feeds /panda/curr_pose, which is what lets active_localizer tell "the arm
is away from home so a miss means I cannot see" apart from "the object is gone".
Without a pose feed that check is disabled, so running this makes the greying-out
behaviour on the dashboard work rather than merely exist.

Deliberately read-only: no controller is started, no stiffness is set, the
brakes are not released and the gripper is not touched. It reads the same
panda_py getters panda.py reads, so the transform is identical to the one skills
are recorded and played against -- a second implementation of the pose would be
a second thing to keep in sync, and a silent source of offsets.

EXCLUSIVE WITH panda.py: libfranka permits a single FCI connection at a time, so
this must not run while the full panda node does. It checks for that on startup
and exits instead of failing obscurely deep inside libfranka. Stop it before
launching a record or play session.

    ros2 run panda_control panda_idle
"""

import sys
import time

import rclpy
from geometry_msgs.msg import Pose, Point, PoseStamped, Quaternion, TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

import panda_py

from panda_control.panda import HOSTNAME

# Frames must match what object_localization looks up; see localizer_service's
# ROBOT_BASE_TF_FRAME / CAMERA_TF_FRAME.
BASE_FRAME = "panda_link0"
HAND_FRAME = "panda_hand"
CURR_POSE_TOPIC = "/panda/curr_pose"


class PandaIdleNode(Node):
    """Publishes where the arm is. Does not move it."""

    def __init__(self) -> None:
        super().__init__("panda_idle")

        # 100 Hz by default, which sounds like a lot for an arm that is parked but
        # is not about smoothness -- it is about the hole between samples. A
        # broadcast at N Hz means the newest transform can be 1/N s old, and tf2
        # will not extrapolate forward, so any image whose pipeline latency is
        # shorter than that hole gets a stamp tf2 refuses to answer for. Measured
        # on this rig the camera's stamps are 25-50 ms old, so 10 Hz (a 100 ms
        # hole) failed most frames while 100 Hz (10 ms) always covers them.
        # panda.py's own 1 Hz broadcast is hopeless for this and is why the scene
        # only ever appeared in bursts.
        self.declare_parameter("rate", 100.0)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.curr_pose_pub = self.create_publisher(PoseStamped, CURR_POSE_TOPIC, 5)

        self.get_logger().info(f"connecting to the arm at {HOSTNAME} (read only)")
        # No Desk.unlock() and no activate_fci() on purpose: unlocking releases
        # the brakes, which is a physical action this node has no business taking.
        # If FCI has never been activated on this boot, run the full panda node
        # once (or activate it from Desk) and then come back.
        self.panda = panda_py.Panda(HOSTNAME)
        self.panda.disable_logging()

        period = 1.0 / max(float(self.get_parameter("rate").value), 0.1)
        self._timer = self.create_timer(period, self.publish_pose)
        self.get_logger().info(
            f"publishing {BASE_FRAME} -> {HAND_FRAME} and {CURR_POSE_TOPIC} "
            f"at {1.0 / period:.1f} Hz")

    def publish_pose(self):
        try:
            position = self.panda.get_position()
            orientation = self.panda.get_orientation(scalar_first=False)  # xyzw
        except Exception as error:  # noqa: BLE001 -- a read failure must not kill the timer
            self.get_logger().warning(f"could not read the robot state: {error}")
            return

        stamp = self.get_clock().now().to_msg()

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = BASE_FRAME
        transform.child_frame_id = HAND_FRAME
        transform.transform.translation.x = float(position[0])
        transform.transform.translation.y = float(position[1])
        transform.transform.translation.z = float(position[2])
        transform.transform.rotation.x = float(orientation[0])
        transform.transform.rotation.y = float(orientation[1])
        transform.transform.rotation.z = float(orientation[2])
        transform.transform.rotation.w = float(orientation[3])
        self.tf_broadcaster.sendTransform(transform)

        self.curr_pose_pub.publish(PoseStamped(
            header=transform.header,
            pose=Pose(
                position=Point(x=float(position[0]), y=float(position[1]), z=float(position[2])),
                orientation=Quaternion(x=float(orientation[0]), y=float(orientation[1]),
                                       z=float(orientation[2]), w=float(orientation[3])),
            ),
        ))


def already_running(node):
    """Whether something is already feeding the arm pose.

    Cheaper to answer here than to let libfranka refuse the second FCI
    connection: that surfaces as a connection error with nothing to say about the
    real cause, which is that the full panda node owns the robot.
    """
    return node.count_publishers(CURR_POSE_TOPIC) > 0


def main():
    rclpy.init()

    # A throwaway node to look at the graph before touching the robot.
    probe = rclpy.create_node("panda_idle_probe")
    time.sleep(1.0)  # discovery needs a moment before counts are meaningful
    conflict = already_running(probe)
    probe.destroy_node()

    if conflict:
        print(f"panda_idle: something already publishes {CURR_POSE_TOPIC} -- the full "
              f"panda node is running, so the arm transform is already available and "
              f"this node is not needed. Exiting.", flush=True)
        rclpy.shutdown()
        return 0

    try:
        node = PandaIdleNode()
    except Exception as error:  # noqa: BLE001
        print(f"panda_idle: could not connect to the arm at {HOSTNAME}: {error}\n"
              f"  - is the robot on and reachable?\n"
              f"  - is FCI active? panda.py activates it via Desk; this node will not,\n"
              f"    because doing so also unlocks the brakes.", flush=True)
        rclpy.shutdown()
        return 1

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
