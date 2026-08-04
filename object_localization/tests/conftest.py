"""Shared ROS setup for the object_localization test suite.

The main thing this file does is move the tests onto their own ROS_DOMAIN_ID
*before* rclpy is initialised.

Why: LocalizationService subscribes to /camera/color/camera_info and to TF, and
on the default domain a running RealSense publishes the former continuously
while static_transform_camera broadcasts part of the latter. So a live rig can
overwrite what a test injected, mid-test, at any moment. That is not
hypothetical -- the first draft of test_get_scene.py silently used the live
camera's intrinsics instead of its own and asserted on the wrong numbers.

The suite does currently pass on the default domain as well, because the values
hardcoded in test_get_scene.py were read off this very rig and the arm is not
broadcasting panda_link0 -> panda_hand. Both coincidences: a recalibration, a
second camera, or panda.py running would each break it, and the failure would
look like a localization bug rather than a test-isolation bug. Hence the domain.

Override with OBJECT_LOCALIZATION_TEST_DOMAIN_ID if 97 collides with something.
"""

import os

# Must happen at import time: the rmw layer reads these when the rclpy context
# is created, and the ros_context fixture below does that.
#
# Derived from the pid rather than fixed, so two test processes cannot meet. They
# otherwise can: a run whose ActiveLocalizerNode is still shutting down will
# happily call the *next* run's compute_object_positions, which showed up once as
# a foreign TF lookup failing an assertion about which stamp was asked for. The
# range keeps clear of 0 (the default anybody's rig uses) and stays inside the
# 0-101 that works without tuning the DDS transport.
os.environ["ROS_DOMAIN_ID"] = os.environ.get(
    "OBJECT_LOCALIZATION_TEST_DOMAIN_ID", str(40 + os.getpid() % 60))
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import threading  # noqa: E402  -- deliberately after the env setup above

import pytest  # noqa: E402
import rclpy  # noqa: E402


@pytest.fixture(scope="session", autouse=True)
def ros_context():
    """Initialise rclpy exactly once for the whole pytest session.

    Teardown shuts rclpy down and waits for background spin threads to notice:
    SpinningRosNode's thread loops on rclpy.ok(), and a spin thread still alive
    while the rclpy C layer finalizes aborts the interpreter (SIGABRT, exit code
    134), which test runners report as a failure after all tests passed.
    """
    rclpy.init()
    yield
    try:
        rclpy.shutdown()
    except Exception:
        pass
    for thread in threading.enumerate():
        if thread is not threading.main_thread():
            thread.join(timeout=2.0)
