"""Tests for the get_scene feature of localizer_service.py.

The feature, as specified: with the camera at home (x=0.4, y=0.0, z=0.4, pointing
down), report the base-frame pose of every object whose template is saved under
cfg/, so that active_localizer can publish them on /scene.

How the pipeline actually splits, which is why the tests below split the same way:

    image + TF ->  LocalizationService.get_scene()      <- the `compute_object_positions` service
                   (SIFT match -> pixel -> base pose)
               ->  ActiveLocalizerNode.publish_scene_thread()
                   (poses -> Scene message)             <- what lands on /scene

The hard part to test is that there is no ground truth: nobody measured where the
robothonboard actually sat when its template was captured. So the suite leans on
properties that cannot be satisfied by accident:

* Self-consistency -- feeding a template's own capture image back in must
  recover the crop-box centre it was drawn around.
* Equivariance -- shifting the image by (du, dv) must move the reported base
  position exactly as geometry.pixel_to_base predicts for the shifted pixel.
  A sign, transpose or frame error survives a "does it land on the table" check
  (z_plane is forced, so it always does) but cannot survive this.
* Invariance -- making the object look bigger must NOT move it, because
  position comes from the ray/plane intersection and never from SIFT scale.

Run from the package root:
    python3 -m pytest tests/test_get_scene.py -v
"""

import os
import time
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import tf_transformations as tft
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from lfd_msgs.srv import GetScene
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

import object_localization
from panda_control.home_pose import HOME_POSE
from object_localization.geometry import pixel_to_base
from object_localization.active_localizer import SCENE_HOME_POSITION
from object_localization.localizer_service import (
    LocalizationService,
    ROBOT_BASE_TF_FRAME,
    SCENE_FRAME_ID,
    CAMERA_TF_FRAME,
)

PACKAGE_PATH = object_localization.package_path
TEMPLATE = "robothonboard"

# The pose the spec calls "home": 0.4 m out, on the centreline, 0.4 m up, with
# the hand rotated 180 deg about base x so the tool -- and the camera bolted next
# to it -- looks down at the table.
HOME_POSITION = list(HOME_POSE.position)
HOME_ORIENTATION = [*HOME_POSE.orientation_wxyz[1:],
                    HOME_POSE.orientation_wxyz[0]]  # xyzw
# active_localizer's scene_home_tolerance default: how far the arm may sit from
# the capture pose before a miss is read as "cannot see" instead of "gone".
SCENE_HOME_TOLERANCE = 0.05

# Real D4xx colour intrinsics for the 1280x720 stream camera_launch.py requests,
# read off /camera/color/camera_info on the rig. Hardcoded rather than
# subscribed: see tests/conftest.py for why a live camera must not reach these
# tests. Update alongside a recalibration.
CAMERA_INFO_K = [644.1641235351562, 0.0, 645.5545654296875,
                 0.0, 643.1190795898438, 363.1338195800781,
                 0.0, 0.0, 1.0]
CAMERA_INFO_D = [-0.05457346513867378, 0.06418613344430923,
                 -0.0008546982426196337, 0.0006812335341237485,
                 -0.021332764998078346]


def _template_params(name=TEMPLATE):
    with open(f"{PACKAGE_PATH}/cfg/{name}/params.yaml") as handle:
        return yaml.safe_load(handle)


def _hand_to_camera():
    """The extrinsics static_transform_camera.py broadcasts, from its own yaml."""
    with open(f"{PACKAGE_PATH}/config/camera_transform.yaml") as handle:
        extrinsics = yaml.safe_load(handle)["camera_transform"]
    return extrinsics["position"], extrinsics["orientation"]


def _matrix(translation, rotation_xyzw):
    return (tft.translation_matrix(translation)
            @ tft.quaternion_matrix(rotation_xyzw))


def _home_camera_in_base():
    """T_base<-camera with the arm at home. The premise of every position below."""
    hand_camera_t, hand_camera_r = _hand_to_camera()
    return (_matrix(HOME_POSITION, HOME_ORIENTATION)
            @ _matrix(hand_camera_t, hand_camera_r))


T_BASE_CAM_HOME = _home_camera_in_base()
K = np.array(CAMERA_INFO_K).reshape(3, 3)
D = np.array(CAMERA_INFO_D)

PARAMS = _template_params()
CROP = PARAMS["crop"]
# Centre of the box drawn around the object at capture time, in full-template
# pixels. measure_object() maps exactly this point through the fitted affine, so
# it is the pixel every expected position below is computed from.
CROP_CENTRE = (0.5 * (CROP[0] + CROP[1]), 0.5 * (CROP[2] + CROP[3]))
CAPTURE_IMAGE_PATH = f"{PACKAGE_PATH}/cfg/{TEMPLATE}/full_image.png"


# --------------------------------------------------------------------------
# fixtures
# --------------------------------------------------------------------------

@pytest.fixture(scope="module")
def tf_publisher():
    """Broadcast the home TF chain: base -> panda_hand -> camera optical frame.

    Static (i.e. timeless) on purpose, so a lookup at any image stamp resolves.
    The service's own TF path is then exercised for real rather than stubbed --
    that chain is half of what "camera at home" means.
    """
    node = Node("object_localization_test_tf")
    broadcaster = tf2_ros.StaticTransformBroadcaster(node)
    hand_camera_t, hand_camera_r = _hand_to_camera()

    def stamped(parent, child, translation, rotation):
        message = TransformStamped()
        message.header.frame_id = parent
        message.header.stamp = node.get_clock().now().to_msg()
        message.child_frame_id = child
        (message.transform.translation.x,
         message.transform.translation.y,
         message.transform.translation.z) = [float(v) for v in translation]
        (message.transform.rotation.x,
         message.transform.rotation.y,
         message.transform.rotation.z,
         message.transform.rotation.w) = [float(v) for v in rotation]
        return message

    broadcaster.sendTransform([
        stamped(ROBOT_BASE_TF_FRAME, "panda_hand", HOME_POSITION, HOME_ORIENTATION),
        stamped("panda_hand", CAMERA_TF_FRAME, hand_camera_t, hand_camera_r),
    ])
    yield node
    node.destroy_node()


@pytest.fixture(scope="module")
def service(tf_publisher):
    """One LocalizationService for the module.

    Module scoped because the constructor sleeps a second and starts a spin
    thread that lives until rclpy shuts down; per-test nodes would add both costs
    twenty times over. Per-test state is reset by the `scene` fixture.
    """
    node = LocalizationService()
    node.camera_info_msg = _camera_info()

    # The TF buffer is filled by the node's own spin thread, so it needs a moment
    # of real time before the chain is complete. Failing here rather than in the
    # first test keeps a TF problem from looking like a localization problem.
    deadline = 200
    while deadline and node._camera_in_base(_now(node)) is None:
        _spin_wait()
        deadline -= 1
    assert deadline, "static TF never reached the service's buffer"
    yield node
    node.destroy_node()


@pytest.fixture
def scene(service):
    """The service with default parameters and a clean localizer cache."""
    service.camera_info_msg = _camera_info()
    service._scene_localizers.clear()
    service.set_parameters([
        Parameter("z_plane", Parameter.Type.DOUBLE, 0.0),
        Parameter("min_inliers", Parameter.Type.INTEGER, 10),
    ])
    return service


def _camera_info():
    message = CameraInfo()
    message.height, message.width = 720, 1280
    message.distortion_model = "plumb_bob"
    message.k = [float(v) for v in CAMERA_INFO_K]
    message.d = [float(v) for v in CAMERA_INFO_D]
    return message


def _spin_wait(seconds=0.02):
    time.sleep(seconds)


def _now(node):
    return node.get_clock().now().to_msg()


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------

def capture_image():
    """The image the template was recorded from, i.e. the camera at home."""
    image = cv2.imread(CAPTURE_IMAGE_PATH)
    assert image is not None, f"missing {CAPTURE_IMAGE_PATH}"
    return image


def as_image_msg(node, image):
    message = CvBridge().cv2_to_imgmsg(image, "bgr8")
    message.header.stamp = _now(node)
    message.header.frame_id = CAMERA_TF_FRAME
    return message


def call_get_scene(node, image=None, image_msg=None):
    if image_msg is None:
        image_msg = as_image_msg(node, capture_image() if image is None else image)
    return node.get_scene(GetScene.Request(img=image_msg), GetScene.Response())


def warp(image, matrix):
    height, width = image.shape[:2]
    return cv2.warpAffine(image, matrix, (width, height))


def apply_affine(matrix, point):
    moved = np.asarray(matrix, float)[0:2, 0:2] @ np.asarray(point, float)
    return tuple(moved + np.asarray(matrix, float)[0:2, 2])


def expected_position(uv, z_plane=0.0, T_base_cam=None):
    position = pixel_to_base(
        K, D, T_BASE_CAM_HOME if T_base_cam is None else T_base_cam, uv, z_plane)
    assert position is not None
    return position


def reprojection_error(position_base, uv, T_base_cam=None):
    """Pixels between `uv` and where OpenCV re-projects `position_base` to.

    An oracle that shares no code with geometry.py: cv2.projectPoints is
    OpenCV's own forward pinhole-plus-distortion model, so agreeing with it means
    the un-projection got the rotation, the signs and the lens right. This
    matters because expected_position() above uses pixel_to_base as its
    reference, which by construction cannot catch an error *inside*
    pixel_to_base. This can -- a transposed rotation shows up here as ~300 px.
    """
    T_camera_base = np.linalg.inv(T_BASE_CAM_HOME if T_base_cam is None else T_base_cam)
    rotation_vector, _ = cv2.Rodrigues(T_camera_base[0:3, 0:3])
    projected, _ = cv2.projectPoints(
        np.array([np.asarray(position_base, dtype=np.float64)]),
        rotation_vector, T_camera_base[0:3, 3], K, D.reshape(1, -1))
    return float(np.linalg.norm(projected[0, 0] - np.asarray(uv, dtype=np.float64)))


def only_pose(response, name=TEMPLATE):
    assert list(response.names) == [name], f"expected just {name}, got {list(response.names)}"
    return response.pose[0]


def position_of(pose_stamped):
    return np.array([pose_stamped.pose.position.x,
                     pose_stamped.pose.position.y,
                     pose_stamped.pose.position.z])


def yaw_of(pose_stamped):
    orientation = pose_stamped.pose.orientation
    return tft.euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w])[2]


# --------------------------------------------------------------------------
# A. the premise: camera at home, pointing down
# --------------------------------------------------------------------------

def test_saved_template_was_captured_near_home():
    """cfg/<template>/params.yaml must record roughly the home pose the spec names.

    Tolerances are loose on purpose. Panda.home() is an IK move followed by an
    impedance controller settling, so it lands within centimetres of nominal
    rather than on it -- the template in the repo sits ~9 mm out. What this
    guards is that a template was not re-recorded from a genuinely different
    viewpoint (another height, or looking sideways), because every expected
    position in this file is computed from the *nominal* home transform.

    A capture pose a centimetre off costs a couple of mm of reported position:
    the optical axis is tilted ~15 deg, so pose error mostly slides x,y, at
    roughly tan(15 deg) of the error. See
    test_camera_height_error_slides_the_object_sideways.
    """
    position = PARAMS["position"]
    offset = np.linalg.norm(
        np.array([position["x"], position["y"], position["z"]]) - np.array(HOME_POSITION))
    # Bounded by active_localizer's scene_home_tolerance rather than by a tighter
    # number of my choosing: that is the offset which actually breaks something.
    # Beyond it the arm sitting at the very pose the template was captured from
    # would be judged "away from home", and the scene would be held stale forever.
    # The template in the repo is ~11 mm out, which is ordinary settling.
    assert offset < SCENE_HOME_TOLERANCE, (
        f"captured {1000 * offset:.0f} mm from nominal home, outside the "
        f"{1000 * SCENE_HOME_TOLERANCE:.0f} mm active_localizer will trust")

    orientation = PARAMS["orientation"]
    recorded = np.array([orientation["x"], orientation["y"],
                         orientation["z"], orientation["w"]])
    # Angle between the two rotations, via |<q1, q2>|. Absolute value because q
    # and -q are the same rotation.
    cosine = min(abs(float(np.dot(recorded, np.array(HOME_ORIENTATION)))), 1.0)
    degrees = np.rad2deg(2.0 * np.arccos(cosine))
    assert degrees < 5.0, f"captured {degrees:.1f} deg away from pointing straight down"


def test_home_camera_is_above_the_table_looking_down():
    assert T_BASE_CAM_HOME[2, 3] > 0.4, "camera should sit above the table"
    assert T_BASE_CAM_HOME[2, 2] < -0.9, "optical +z should point down in base"


def test_tf_chain_reproduces_the_home_transform(service):
    """The service's own TF lookup must agree with the transform built by hand.

    Catches an inverted edge or a source/target swap in _camera_in_base, which
    would otherwise show up only as objects in implausible places.
    """
    resolved = service._camera_in_base(_now(service))
    assert resolved is not None
    assert np.allclose(resolved, T_BASE_CAM_HOME, atol=1e-9)


def test_transform_is_looked_up_at_the_image_stamp(scene, monkeypatch):
    """Not at "latest". The camera rides on the end effector, so a transform
    newer than the picture puts objects wherever the arm has since moved to.

    Asserted on the request rather than the result because the test rig
    broadcasts static (timeless) transforms, for which both answers agree -- only
    a moving arm tells them apart, and that is exactly the case this protects.
    """
    asked_for = []
    original = scene.lookup_relative_transform

    def recording(source_frame, target_frame, at_time=None, **kwargs):
        asked_for.append(at_time)
        return original(source_frame, target_frame, at_time=at_time, **kwargs)

    monkeypatch.setattr(scene, "lookup_relative_transform", recording)
    image_msg = as_image_msg(scene, capture_image())
    call_get_scene(scene, image_msg=image_msg)

    assert asked_for, "get_scene resolved the camera pose without asking TF"
    stamp = Time.from_msg(image_msg.header.stamp)
    # "no lookup asked for latest" plus "ours asked for the image stamp", rather
    # than "every lookup used our stamp": the service is shared, so a concurrent
    # caller's lookup may legitimately appear in this list.
    # Identity, not `None in asked_for`: rclpy's Time.__eq__ raises TypeError on a
    # non-Time operand rather than returning False, so `in` would blow up here.
    # This assertion must also come first, for the same reason.
    assert all(at_time is not None for at_time in asked_for), (
        "a lookup asked for the latest transform instead of the image stamp")
    assert any(at_time == stamp for at_time in asked_for), (
        f"looked up at {asked_for}, expected one at the image stamp {stamp}")


def _buffer_node_with(parent, child, stamp):
    """A CustomTransformListener node holding exactly one transform, at `stamp`.

    Injected with Buffer.set_transform rather than broadcast: no networking, no
    spin thread, no race. Frame names are test-only so this cannot disturb the
    home chain the rest of the module relies on.
    """
    from object_localization.tf_utils import CustomTransformListener

    # Mirrors how LocalizationService composes these: CustomTransformListener
    # calls super().__init__() with no arguments, so whatever follows it in the
    # MRO has to supply the node name itself.
    class _Named(Node):
        def __init__(self):
            super().__init__(f"tf_tolerance_probe_{os.getpid()}_{stamp.nanoseconds}")

    class _Probe(CustomTransformListener, _Named):
        pass

    node = _Probe()
    message = TransformStamped()
    message.header.stamp = stamp.to_msg()
    message.header.frame_id = parent
    message.child_frame_id = child
    message.transform.rotation.w = 1.0
    node.tf_buffer.set_transform(message, "unittest")
    return node


def test_a_stamp_just_past_the_newest_transform_is_tolerated():
    """The failure that made the scene appear in bursts.

    A broadcaster at N Hz leaves a 1/N second hole after each sample. The camera's
    stamps are 25-50 ms old on this rig, so at 10 Hz they land inside that hole,
    tf2 refuses to extrapolate forward, and the frame is dropped even though the
    arm's pose is known to within milliseconds. Within the tolerance the newest
    sample is used instead; past it the lookup still fails, because the point of
    asking at a stamp is not to silently use a pose from somewhere else.
    """
    from rclpy.duration import Duration

    from rclpy.clock import ClockType

    # ROS_TIME to match what Time.from_msg(image.header.stamp) produces.
    base = Time(seconds=1000, nanoseconds=0, clock_type=ClockType.ROS_TIME)
    node = _buffer_node_with("tol_parent", "tol_child", base)
    try:
        asked_at = base + Duration(seconds=0.03)  # 30 ms past the only sample

        # Without a tolerance this is exactly the reported failure.
        translation, _ = node.lookup_relative_transform(
            "tol_parent", "tol_child", at_time=asked_at)
        assert translation is None, "extrapolating into the future should fail"

        translation, _ = node.lookup_relative_transform(
            "tol_parent", "tol_child", at_time=asked_at,
            future_tolerance=Duration(seconds=0.05))
        assert translation is not None, "30 ms inside a 50 ms tolerance must resolve"

        translation, _ = node.lookup_relative_transform(
            "tol_parent", "tol_child", at_time=asked_at,
            future_tolerance=Duration(seconds=0.01))
        assert translation is None, "30 ms outside a 10 ms tolerance must still fail"
    finally:
        node.destroy_node()


def test_tolerance_does_not_invent_an_unknown_frame():
    """Adding the tolerance must not turn "I have never heard of that frame" into
    an answer.

    Note this exercises the *outer* handler, not the fallback: an unknown frame
    raises LookupException rather than ExtrapolationException, so _latest_within
    is never consulted. Its own "latest also failed" branch is defensive against
    the buffer being cleared between the two lookups, and is not reachable from a
    test -- if extrapolation was the complaint, a latest lookup will succeed.
    """
    from rclpy.duration import Duration

    node = _buffer_node_with("tol_parent", "tol_child", Time(seconds=1000))
    try:
        translation, _ = node.lookup_relative_transform(
            "tol_parent", "absent_frame", at_time=Time(seconds=1000),
            future_tolerance=Duration(seconds=10.0))
        assert translation is None
    finally:
        node.destroy_node()


def test_scene_lookups_carry_the_future_tolerance(scene, monkeypatch):
    """_camera_in_base must pass the parameter through, not just declare it."""
    from rclpy.duration import Duration

    tolerances = []
    original = scene.lookup_relative_transform

    def recording(source_frame, target_frame, at_time=None, future_tolerance=None):
        tolerances.append(future_tolerance)
        return original(source_frame, target_frame, at_time=at_time,
                        future_tolerance=future_tolerance)

    monkeypatch.setattr(scene, "lookup_relative_transform", recording)
    scene.set_parameters([
        Parameter("tf_future_tolerance", Parameter.Type.DOUBLE, 0.075)])
    call_get_scene(scene)

    assert tolerances, "no TF lookup happened"
    assert all(isinstance(t, Duration) for t in tolerances), (
        f"scene lookups must be tolerant of a just-future stamp, got {tolerances}")
    assert all(t == Duration(seconds=0.075) for t in tolerances)


def test_capture_image_matches_the_stream_camera_launch_requests():
    """Template pixels and camera_info must describe the same frame size.

    A crop recorded at one resolution is meaningless against another, and the
    intrinsics above are for 1280x720.
    """
    height, width = capture_image().shape[:2]
    assert (width, height) == (_camera_info().width, _camera_info().height)


# --------------------------------------------------------------------------
# B. the object is found, and reported the way consumers expect
# --------------------------------------------------------------------------

def test_finds_the_saved_template(scene):
    response = call_get_scene(scene)
    assert list(response.names) == [TEMPLATE]


def test_names_and_poses_stay_parallel(scene):
    """active_localizer zips these two arrays; a length mismatch silently drops
    or mislabels objects rather than failing."""
    response = call_get_scene(scene)
    assert len(response.names) == len(response.pose)


def test_recovers_the_crop_box_centre(scene):
    """Feeding the capture image back must land on the box drawn at capture time.

    This is the whole chain -- template keypoints, affine fit, crop centre,
    un-projection, TF -- checked against a pixel that is known exactly.
    """
    position = position_of(only_pose(call_get_scene(scene)))
    assert np.allclose(position, expected_position(CROP_CENTRE), atol=1e-3)


def test_pose_is_published_in_the_scene_frame(scene):
    """SCENE_FRAME_ID, not the TF frame name: scene_marker_pub and a404.yaml
    call the robot base "base" while panda.py broadcasts "panda_link0"."""
    assert only_pose(call_get_scene(scene)).header.frame_id == SCENE_FRAME_ID


def test_pose_echoes_the_image_stamp(scene):
    """The pose is only as fresh as the picture it came from, and downstream has
    no other way to tell how old it is."""
    image_msg = as_image_msg(scene, capture_image())
    response = call_get_scene(scene, image_msg=image_msg)
    assert only_pose(response).header.stamp == image_msg.header.stamp


def test_lands_exactly_on_the_requested_plane(scene):
    for z_plane in (0.0, 0.04, 0.10):
        scene.set_parameters([Parameter("z_plane", Parameter.Type.DOUBLE, z_plane)])
        position = position_of(only_pose(call_get_scene(scene)))
        assert position[2] == pytest.approx(z_plane, abs=1e-12)
        assert np.allclose(position, expected_position(CROP_CENTRE, z_plane), atol=1e-3)


def test_object_is_within_reach_on_the_table(scene):
    """Envelope check. Weak on its own -- z_plane forces the height, so a badly
    wrong x,y still lands on the table -- but it catches a result that is off by
    a metre or behind the robot."""
    position = position_of(only_pose(call_get_scene(scene)))
    assert 0.2 < position[0] < 0.8, f"x out of the workspace: {position[0]}"
    assert abs(position[1]) < 0.5, f"y out of the workspace: {position[1]}"


def test_repeatable_across_calls(scene):
    """Localizer instances are cached and reused between calls. detect_points()
    clears _src_pts for that reason; if it stopped doing so, a template that
    matched once would keep reporting its old position forever."""
    first = position_of(only_pose(call_get_scene(scene)))
    second = position_of(only_pose(call_get_scene(scene)))
    assert np.allclose(first, second, atol=1e-9)


def test_a_template_that_stops_matching_stops_being_reported(scene):
    """Once matched, a template must not keep reporting that position forever.

    Same cached Localizer for both calls, which is the point: detect_points()
    clears _src_pts precisely so the second call cannot re-measure last frame's
    keypoints. Without that clearing, an object removed from the table stays on
    /scene indefinitely -- and every other test here still passes.
    """
    assert list(call_get_scene(scene).names) == [TEMPLATE], "fixture precondition"
    blank = np.zeros_like(capture_image())
    assert list(call_get_scene(scene, image=blank).names) == []


def test_orientation_is_a_pure_yaw(scene):
    """The object lies on the table, so only rotation about base z is meaningful
    -- get_scene builds the quaternion from (0, 0, yaw)."""
    orientation = only_pose(call_get_scene(scene)).pose.orientation
    roll, pitch, _ = tft.euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w])
    assert roll == pytest.approx(0.0, abs=1e-9)
    assert pitch == pytest.approx(0.0, abs=1e-9)
    norm = np.linalg.norm([orientation.x, orientation.y, orientation.z, orientation.w])
    assert norm == pytest.approx(1.0, abs=1e-9)


# --------------------------------------------------------------------------
# C. equivariance: move the object, the pose must move to match
# --------------------------------------------------------------------------

@pytest.mark.parametrize("du,dv", [(40, 25), (-60, 30), (0, -50), (80, 0)])
def test_moving_the_object_moves_the_pose_as_geometry_predicts(scene, du, dv):
    """Shift the picture, and the reported pose must follow the shifted pixel.

    The strongest test here. It needs no ground truth, and no sign or transpose
    error in the pixel -> base chain can satisfy it, because the predicted
    position is recomputed from the moved pixel for every offset.
    """
    shift = np.float32([[1, 0, du], [0, 1, dv]])
    response = call_get_scene(scene, image=warp(capture_image(), shift))
    position = position_of(only_pose(response))
    assert np.allclose(position, expected_position(apply_affine(shift, CROP_CENTRE)),
                       atol=2e-3)


@pytest.mark.parametrize("du,dv", [(0, 0), (40, 25), (-60, 30)])
def test_reported_pose_reprojects_onto_the_matched_pixel(scene, du, dv):
    """Independent check on the same claim, via OpenCV's forward projection.

    Sends the published base-frame position back through cv2.projectPoints and
    requires it to land on the pixel the match reported. Shares no code with
    geometry.py, so unlike the test above it also fails when the un-projection
    itself is wrong rather than just mis-wired.
    """
    shift = np.float32([[1, 0, du], [0, 1, dv]])
    response = call_get_scene(scene, image=warp(capture_image(), shift))
    position = position_of(only_pose(response))
    error = reprojection_error(position, apply_affine(shift, CROP_CENTRE))
    assert error < 1.0, f"published pose re-projects {error:.1f} px away"


def test_a_shifted_object_actually_moves(scene):
    """Guard for the test above: prove the offsets are big enough to matter, so
    a pipeline that ignored the image entirely could not pass by returning a
    constant."""
    still = position_of(only_pose(call_get_scene(scene)))
    shifted = position_of(only_pose(call_get_scene(
        scene, image=warp(capture_image(), np.float32([[1, 0, 80], [0, 1, 0]])))))
    assert np.linalg.norm(shifted - still) > 0.02


@pytest.mark.parametrize("degrees", [10, -25, 40])
def test_turning_the_object_turns_the_reported_yaw(scene, degrees):
    """Rotating about the crop centre must change yaw and leave position put.

    The sign matters and is easy to get backwards: the image rotation is about
    the optical axis, which points *down*, so yaw_in_base flips it.
    """
    rotation = cv2.getRotationMatrix2D(CROP_CENTRE, degrees, 1.0)
    pose = only_pose(call_get_scene(scene, image=warp(capture_image(), rotation)))

    assert yaw_of(pose) == pytest.approx(np.deg2rad(degrees), abs=np.deg2rad(1.0))
    assert np.allclose(position_of(pose), expected_position(CROP_CENTRE), atol=3e-3)


@pytest.mark.parametrize("factor", [0.8, 1.25])
def test_apparent_size_does_not_move_the_object(scene, factor):
    """Position comes from the ray/plane intersection, never from SIFT scale.

    Pinning the documented design choice: scale is reported as a diagnostic
    only, so an object that merely *looks* bigger must not move.
    """
    zoom = cv2.getRotationMatrix2D(CROP_CENTRE, 0.0, factor)
    position = position_of(only_pose(call_get_scene(scene, image=warp(capture_image(), zoom))))
    assert np.allclose(position, expected_position(CROP_CENTRE), atol=3e-3)


def test_camera_height_error_slides_the_object_sideways(scene, monkeypatch):
    """Documents the failure mode to expect from bad extrinsics or z_plane.

    The optical axis is tilted ~15 deg, so a height error does not read as a
    height error: it slides x,y. This is where the ~80-95 mm range discrepancy
    logged per object would land if it lives in the extrinsics.
    """
    truth = position_of(only_pose(call_get_scene(scene)))

    lifted = T_BASE_CAM_HOME.copy()
    lifted[2, 3] += 0.070
    monkeypatch.setattr(scene, "_camera_in_base", lambda stamp: lifted)
    biased = position_of(only_pose(call_get_scene(scene)))

    lateral = np.linalg.norm(biased[:2] - truth[:2])
    assert 0.010 < lateral < 0.030, f"expected ~19 mm of slide, got {1000 * lateral:.1f} mm"


# --------------------------------------------------------------------------
# D. degenerate inputs must come back empty, not wrong
# --------------------------------------------------------------------------

def test_no_camera_info_yields_no_objects(scene):
    """Without intrinsics there is nothing to un-project with. Publishing a
    guessed pose would be worse than publishing none."""
    scene.camera_info_msg = None
    response = call_get_scene(scene)
    assert list(response.names) == [] and list(response.pose) == []


def test_no_transform_yields_no_objects(scene, monkeypatch):
    """The camera rides on the end effector, so with no TF at the image's stamp
    the pixel cannot be placed in the base frame at all."""
    monkeypatch.setattr(scene, "_camera_in_base", lambda stamp: None)
    response = call_get_scene(scene)
    assert list(response.names) == [] and list(response.pose) == []


def test_blank_image_yields_no_objects(scene):
    response = call_get_scene(scene, image=np.zeros_like(capture_image()))
    assert list(response.names) == []


def test_noise_image_yields_no_objects(scene):
    """Texture everywhere, structure nowhere: RANSAC must not fit a pose to it."""
    generator = np.random.default_rng(seed=0)
    noise = generator.integers(0, 256, capture_image().shape, dtype=np.uint8)
    response = call_get_scene(scene, image=noise)
    assert list(response.names) == []


def test_min_inliers_rejects_a_weak_match(scene):
    """The threshold has to actually gate publishing, otherwise a few stray
    keypoints put a phantom object on /scene."""
    unreachable = 10_000
    scene.set_parameters([Parameter("min_inliers", Parameter.Type.INTEGER, unreachable)])
    assert list(call_get_scene(scene).names) == []

    scene.set_parameters([Parameter("min_inliers", Parameter.Type.INTEGER, 10)])
    assert list(call_get_scene(scene).names) == [TEMPLATE]


def test_plane_the_ray_cannot_reach_yields_no_objects(scene):
    """A plane above the camera: the downward ray never gets there. get_scene
    must skip the object rather than extrapolate backwards through the lens."""
    scene.set_parameters([Parameter("z_plane", Parameter.Type.DOUBLE, 5.0)])
    assert list(call_get_scene(scene).names) == []


def test_camera_looking_sideways_yields_no_objects(scene, monkeypatch):
    """Ray parallel to the table has no intersection to report."""
    sideways = T_BASE_CAM_HOME.copy()
    sideways[0:3, 0:3] = tft.euler_matrix(0.0, np.pi / 2, 0.0)[0:3, 0:3]
    monkeypatch.setattr(scene, "_camera_in_base", lambda stamp: sideways)
    assert list(call_get_scene(scene).names) == []


# --------------------------------------------------------------------------
# E. the cfg/ listing itself -- "from list of saved configurations in cfg"
# --------------------------------------------------------------------------

@pytest.fixture
def cfg_root(tmp_path, monkeypatch, scene):
    """Point the service at a throwaway cfg/ tree.

    package_path is read at call time inside get_scene, so patching the module
    attribute is enough. The localizer cache is keyed by template name only, so
    it has to be cleared alongside it.
    """
    (tmp_path / "cfg").mkdir()
    monkeypatch.setattr(object_localization, "package_path", str(tmp_path))
    scene._scene_localizers.clear()

    def add_template(name, with_params=True):
        folder = tmp_path / "cfg" / name
        folder.mkdir()
        # Symlinked, not copied: full_image.png is 1.1 MB and read-only here.
        os.symlink(CAPTURE_IMAGE_PATH, folder / "full_image.png")
        if with_params:
            with open(folder / "params.yaml", "w") as handle:
                yaml.safe_dump(PARAMS, handle)
        return folder

    yield add_template
    scene._scene_localizers.clear()


def test_every_saved_template_is_reported(scene, cfg_root):
    """Two saved configurations, two objects. get_scene walks cfg/ rather than
    taking a name, so adding a template is all it should take to see it."""
    cfg_root("board_left")
    cfg_root("board_right")
    response = call_get_scene(scene)
    assert list(response.names) == ["board_left", "board_right"]
    assert len(response.pose) == 2


def test_reported_in_sorted_order(scene, cfg_root):
    """os.walk order is filesystem-dependent; get_scene sorts so that names and
    poses line up run to run."""
    for name in ("zebra", "apple", "mango"):
        cfg_root(name)
    assert list(call_get_scene(scene).names) == ["apple", "mango", "zebra"]


def test_template_without_params_is_skipped(scene, cfg_root):
    """A folder holding only an image is a half-recorded template: no crop box,
    no capture range. Skip it instead of crashing the whole scene."""
    cfg_root("complete")
    cfg_root("no_params", with_params=False)
    assert list(call_get_scene(scene).names) == ["complete"]


def test_empty_cfg_yields_no_objects(scene, cfg_root):
    assert list(call_get_scene(scene).names) == []


def test_template_names_are_unique(scene, cfg_root):
    """Scene() rejects duplicate object names, so a repeated name would make
    active_localizer's publish throw. Template names come from folder names,
    which the filesystem already keeps unique -- this pins that reasoning."""
    cfg_root("board_left")
    cfg_root("board_right")
    names = list(call_get_scene(scene).names)
    assert len(names) == len(set(names))


# --------------------------------------------------------------------------
# F. the /scene leg: poses -> Scene message
# --------------------------------------------------------------------------

def test_get_scene_response_survives_the_trip_to_a_scene_message(scene):
    """The conversion active_localizer.publish_scene_thread does, in isolation.

    Deliberately not routed through ActiveLocalizerNode.transform(): that is the
    servoing helper, and it clamps z to the home EE height and flattens
    orientation, which would undo everything get_scene just computed.
    """
    from scene_getter.scene_lib.scene import Scene
    from scene_getter.scene_lib.scene_object import SceneObject

    response = call_get_scene(scene)
    assert len(response.names) == 1, "fixture precondition"

    objects = [
        SceneObject.from_dict(name, {
            "position": [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
            "orientation": [pose.pose.orientation.x, pose.pose.orientation.y,
                            pose.pose.orientation.z, pose.pose.orientation.w],
            "params": "",
        })
        for name, pose in zip(response.names, response.pose)
    ]
    message = Scene(name="active_localizer_scene", objects=objects).to_ros()

    assert [obj.name for obj in message.objects] == [TEMPLATE]
    published = message.objects[0]
    expected = position_of(response.pose[0])
    assert (published.pose.position.x, published.pose.position.y,
            published.pose.position.z) == pytest.approx(tuple(expected), abs=1e-12)
    assert published.pose.orientation == response.pose[0].pose.orientation


def test_scene_message_keeps_the_base_frame_position(scene):
    """The pose that reaches /scene must be the base-frame one get_scene
    produced -- not re-transformed, not height-clamped."""
    from scene_getter.scene_lib.scene import Scene
    from scene_getter.scene_lib.scene_object import SceneObject

    response = call_get_scene(scene)
    expected = position_of(response.pose[0])
    scene_message = Scene(name="s", objects=[
        SceneObject.from_dict(response.names[0], {
            "position": list(expected),
            "orientation": [0.0, 0.0, 0.0, 1.0],
            "params": "",
        })]).to_ros()

    recovered = Scene.from_ros(scene_message).objects[0].position
    assert np.allclose(recovered, expected, atol=1e-12)
    assert recovered[2] == pytest.approx(0.0, abs=1e-12), "table height must survive"


# --------------------------------------------------------------------------
# G. end to end: a message actually arrives on /scene
#
# Kept last in the file. ActiveLocalizerNode's publisher is a bare daemon thread
# that calls compute_object_positions once a second for the life of the process;
# the fixture switches it off on teardown so it cannot mutate the shared
# service's cached Localizers underneath a later test.
# --------------------------------------------------------------------------

@pytest.fixture
def scene_topic(scene):
    """LocalizationService + ActiveLocalizerNode + an image feed and a /scene ear.

    Deliberately never sets publishing_scene. Everything here relies on the
    listener's subscription being the request to publish, which is the design
    under test.
    """
    from object_localization.active_localizer import ActiveLocalizerNode
    from rclpy.executors import SingleThreadedExecutor
    import scene_msgs.msg as scene_msgs
    import threading

    # ActiveLocalizerNode blocks in __init__ until compute_localization answers,
    # which the `scene` fixture's service provides.
    active = ActiveLocalizerNode()

    received = []
    listener = Node("object_localization_test_scene_listener")
    listener.create_subscription(
        scene_msgs.Scene, "/scene", lambda message: received.append(message), 5)
    image_publisher = listener.create_publisher(Image, "/camera/color/image_raw", 5)

    # One executor for both helper nodes: rclpy.spin() falls back to a single
    # global executor, so spinning two nodes with it deadlocks on
    # "generator already executing".
    executor = SingleThreadedExecutor()
    executor.add_node(listener)

    def spin():
        # Same swallow SpinningRosNode._spin does: rclpy.shutdown() at session
        # teardown otherwise dumps an ExternalShutdownException traceback from
        # this thread into an otherwise green run.
        from rclpy.executors import ExternalShutdownException
        try:
            executor.spin()
        except (ExternalShutdownException, RuntimeError):
            pass

    threading.Thread(target=spin, daemon=True).start()

    # 5 Hz so the tests do not spend a second per cycle. One get_scene pass over
    # the single saved template costs ~190 ms, so this is achievable.
    active.set_parameters([Parameter("scene_rate", Parameter.Type.DOUBLE, 5.0)])

    def pump(image, count=1, timeout=30.0):
        """Feed `image` until `count` Scene messages have arrived, or time out."""
        message = CvBridge().cv2_to_imgmsg(image, "bgr8")
        deadline = time.time() + timeout
        while time.time() < deadline and len(received) < count:
            # Restamped every time: publish_scene_thread drops frames older than
            # a second, on the grounds that a stale frame is a stale scene.
            message.header.stamp = listener.get_clock().now().to_msg()
            image_publisher.publish(message)
            time.sleep(0.05)
        return received

    yield SimpleNamespace(pump=pump, node=active, received=received)

    active.publishing_scene = False
    executor.remove_node(listener)
    listener.destroy_node()
    active.destroy_node()


def test_object_poses_reach_the_scene_topic(scene_topic):
    """The headline claim: with the camera at home, objects saved under cfg/ show
    up on /scene in the base frame -- and subscribing is all it takes.

    Everything else in this file tests get_scene directly, which skips the leg
    that actually publishes: the service response could be perfect while nothing
    ever reaches a subscriber. Nothing here calls start_publishing_scene, so this
    also pins the subscriber-driven trigger.
    """
    received = scene_topic.pump(capture_image())
    assert received, "nothing was published on /scene within the timeout"

    objects = received[0].objects
    assert [obj.name for obj in objects] == [TEMPLATE]

    published = np.array([objects[0].pose.position.x,
                          objects[0].pose.position.y,
                          objects[0].pose.position.z])
    assert np.allclose(published, expected_position(CROP_CENTRE), atol=1e-3)
    assert published[2] == pytest.approx(0.0, abs=1e-12), (
        "z must survive the trip: transform() would clamp it to the home EE height")


def test_publishing_is_off_until_asked(scene):
    """start/stop_publishing_scene are the switch lfd.py and
    SceneGetterViaObjectLocalizer throw, and both check .success -- which
    Trigger.Response defaults to False."""
    from object_localization.active_localizer import ActiveLocalizerNode
    from std_srvs.srv import Trigger

    active = ActiveLocalizerNode()
    try:
        assert active.publishing_scene is False, "must not publish unasked"

        assert active.start_publishing_scene(Trigger.Request(), Trigger.Response()).success
        assert active.publishing_scene is True

        assert active.stop_publishing_scene(Trigger.Request(), Trigger.Response()).success
        assert active.publishing_scene is False
    finally:
        active.publishing_scene = False
        active.destroy_node()


def test_scene_is_stamped_when_it_was_seen_not_when_published(scene_topic):
    """header.stamp carries the observation time, which is the only staleness
    signal on the wire.

    SceneObject.params cannot carry it: that field is free-form description text
    that Scene.get_params() joins and hands downstream, so a status flag in it
    would corrupt the description. A consumer greys objects out by age instead.
    """
    received = scene_topic.pump(capture_image())
    assert received

    published = received[0]
    assert published.header.frame_id == SCENE_FRAME_ID
    stamp = published.header.stamp
    assert (stamp.sec, stamp.nanosec) != (0, 0), "header.stamp was never set"

    age = scene_topic.node.get_clock().now() - Time.from_msg(stamp)
    assert 0 <= age.nanoseconds < 5e9, f"stamp is {age.nanoseconds / 1e9:.1f} s off"


def test_off_home_holds_the_last_seen_poses(scene_topic):
    """Away from home the templates stop matching, and a miss means "cannot see".

    Proven with a blank frame: if the home gate works the previously seen poses
    are republished unchanged, and if it does not the blank frame is believed and
    the scene empties.
    """
    assert scene_topic.pump(capture_image()), "nothing observed to begin with"
    seen = scene_topic.received[0]
    seen_names = [obj.name for obj in seen.objects]
    seen_position = seen.objects[0].pose.position

    # Lift the arm well clear of the capture pose, then show it nothing.
    scene_topic.node.curr_pos = [0.4, 0.0, 0.7]
    already = len(scene_topic.received)
    scene_topic.pump(np.zeros_like(capture_image()), count=already + 2)
    assert len(scene_topic.received) > already, "publishing stopped entirely"

    held = scene_topic.received[-1]
    assert [obj.name for obj in held.objects] == seen_names, "objects were dropped"
    assert held.objects[0].pose.position == seen_position, "poses changed while blind"
    assert held.header.stamp == seen.header.stamp, (
        "held poses must keep the stamp they were seen at, otherwise they look fresh")


def test_at_home_an_object_that_stops_matching_disappears(scene_topic):
    """The other half of the gate, and the reason it cannot simply always hold.

    At home a template that stops matching really has been picked up or moved, so
    keeping its last pose would leave a phantom object on /scene for the rest of
    the session.
    """
    assert scene_topic.pump(capture_image()), "nothing observed to begin with"
    scene_topic.node.curr_pos = list(SCENE_HOME_POSITION)

    already = len(scene_topic.received)
    scene_topic.pump(np.zeros_like(capture_image()), count=already + 3)
    assert scene_topic.received[-1].objects == [], (
        "an object removed while the arm was at home stayed on /scene")


def test_nothing_is_computed_while_nobody_listens(scene):
    """No subscriber and no explicit request means no SIFT at all.

    The point of the subscriber-driven trigger: during action execution the
    dashboard is closed, and get_scene must not be competing with the servo loop
    for the localizer's executor.
    """
    from object_localization.active_localizer import ActiveLocalizerNode

    active = ActiveLocalizerNode()
    try:
        active.set_parameters([Parameter("scene_rate", Parameter.Type.DOUBLE, 20.0)])
        looked = []
        active._untrustworthy_view = lambda: looked.append(1) or "stubbed"

        assert active.scene_pub.get_subscription_count() == 0, "test rig has a listener"
        assert active.publishing_scene is False
        time.sleep(1.0)
        assert looked == [], f"did {len(looked)} scene passes with nobody listening"

        # ...and the explicit switch still works for lfd.py, which wants the
        # scene without subscribing first.
        active.publishing_scene = True
        time.sleep(1.0)
        assert looked, "start_publishing_scene no longer forces publishing"
    finally:
        active.publishing_scene = False
        active.destroy_node()


def test_missing_arm_transform_is_named_not_silent(scene, monkeypatch):
    """A missing panda_link0 -> panda_hand must be reported, not read as "empty".

    This is the failure that looks like nothing: panda.py is only launched to
    record or play, so while merely watching /scene there is no arm transform,
    get_scene can only answer with an empty response, and the dashboard shows a
    blank table for no stated reason. The reason has to reach the log, and the
    remembered scene must not be overwritten by it.
    """
    from object_localization.active_localizer import ActiveLocalizerNode

    active = ActiveLocalizerNode()
    try:
        active._img = as_image_msg(scene, capture_image())
        active.img_last_rec = time.time()
        monkeypatch.setattr(active, "lookup_relative_transform",
                            lambda source, target, at_time=None: (None, None))

        reason = active._untrustworthy_view()
        assert reason is not None, "a missing arm transform was treated as trustworthy"
        assert "panda_hand" in reason and "panda" in reason.lower()
        assert "panda_idle" in reason, "the log should say how to fix it"
    finally:
        active.publishing_scene = False
        active.destroy_node()


def test_home_gate_thresholds(scene):
    """_untrustworthy_view's decisions, without the timing of the live thread."""
    from object_localization.active_localizer import ActiveLocalizerNode

    active = ActiveLocalizerNode()
    try:
        assert "no image" in active._untrustworthy_view()

        active._img = as_image_msg(scene, capture_image())
        active.img_last_rec = time.time() - 5.0
        assert "not fresh" in active._untrustworthy_view()

        # The node's own TF buffer is filled by its spin thread, so the arm
        # transform is not there the instant it is constructed. Wait for it, or the
        # checks below race against it and see the transform reason instead.
        active.curr_pos = None
        deadline = 250
        while deadline:
            active.img_last_rec = time.time()
            if active._untrustworthy_view() is None:
                break
            time.sleep(0.02)
            deadline -= 1
        assert deadline, "the arm transform never reached active_localizer's buffer"

        active.img_last_rec = time.time()
        # No pose feed: the arm transform above is the hard requirement, so trying
        # is better than refusing on a missing topic.
        active.curr_pos = None
        assert active._untrustworthy_view() is None

        active.curr_pos = list(SCENE_HOME_POSITION)
        assert active._untrustworthy_view() is None

        tolerance = float(active.get_parameter("scene_home_tolerance").value)
        active.curr_pos = list(SCENE_HOME_POSITION)
        active.curr_pos[0] += tolerance * 0.5
        assert active._untrustworthy_view() is None, "inside tolerance must be trusted"

        active.curr_pos = list(SCENE_HOME_POSITION)
        active.curr_pos[0] += tolerance * 3.0
        reason = active._untrustworthy_view()
        assert reason is not None and "from home" in reason
    finally:
        active.publishing_scene = False
        active.destroy_node()


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
