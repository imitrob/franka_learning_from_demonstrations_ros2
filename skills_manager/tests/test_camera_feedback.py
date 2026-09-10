from types import SimpleNamespace

import numpy as np
from builtin_interfaces.msg import Time

from skills_manager.camera_feedback import CameraFeedback
from skills_manager.transfom import Transform


def test_correction_marker_uses_float_fields():
    published = []
    feedback = CameraFeedback.__new__(CameraFeedback)
    feedback.curr_pos = np.array([0.4, 0.0, 0.3])
    feedback.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time())
    )
    feedback.marker_pub = SimpleNamespace(publish=published.append)

    feedback.publish_correction_marker(np.eye(4))

    assert published[0].scale.z == 0.0


def test_sift_without_features_skips_correction():
    feedback = CameraFeedback.__new__(CameraFeedback)
    feedback.curr_image = np.zeros((720, 1280, 3), dtype=np.uint8)
    feedback.cx_cy_array = np.array([640.0, 360.0])
    feedback.ds_factor = 4
    feedback.row_crop_pct_top = 0.0
    feedback.row_crop_pct_bot = 1.0
    feedback.col_crop_pct_left = 0.0
    feedback.col_crop_pct_right = 1.0
    feedback.loaded_img = np.zeros((1, 180, 320), dtype=np.uint8)
    feedback.time_index = 1
    feedback.camera_correction = np.zeros(3)
    feedback.get_transform = lambda *_args: (_ for _ in ()).throw(
        AssertionError("TF should not be requested without SIFT features")
    )

    feedback.sift_matching()

    np.testing.assert_array_equal(feedback.camera_correction, np.zeros(3))


def test_get_transform_uses_tf2_buffer():
    stamped = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=1.0, y=2.0, z=3.0),
        rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    ))
    transform = Transform.__new__(Transform)
    transform.tf_buffer = SimpleNamespace(
        lookup_transform=lambda *_args, **_kwargs: stamped
    )

    matrix = transform.get_transform("panda_link0", "camera_color_optical_frame")

    np.testing.assert_array_equal(matrix, np.array([
        [1.0, 0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0, 2.0],
        [0.0, 0.0, 1.0, 3.0],
        [0.0, 0.0, 0.0, 1.0],
    ]))


def _feedback_with_shift(shift_px):
    """CameraFeedback stub whose live image is the template shifted by `shift_px` columns."""
    import cv2

    rng = np.random.default_rng(0)
    template = rng.integers(0, 255, (180, 320), dtype=np.uint8)
    shifted = np.roll(template, shift_px, axis=1)
    curr_image = cv2.resize(
        cv2.cvtColor(shifted, cv2.COLOR_GRAY2BGR), (1280, 720), interpolation=cv2.INTER_NEAREST
    )

    feedback = CameraFeedback.__new__(CameraFeedback)
    feedback.curr_image = curr_image
    feedback.cx_cy_array = np.array([640.0, 360.0])
    feedback.ds_factor = 4
    feedback.row_crop_pct_top = 0.0
    feedback.row_crop_pct_bot = 1.0
    feedback.col_crop_pct_left = 0.0
    feedback.col_crop_pct_right = 1.0
    feedback.x_dist_threshold = 2
    feedback.y_dist_threshold = 2
    feedback.num_good_matches_threshold = 6
    feedback.correction_gain = 0.001
    feedback.max_correction_step = 0.005
    feedback.correction_increment = 0.0005
    feedback.sift_converged = True
    feedback.sift_window = 5
    feedback.settle_tolerance = 0.002
    feedback._sift_errors = []
    feedback._last_sift_pos = None
    feedback.loaded_img = template[None, ...]
    feedback.time_index = 1
    feedback.camera_correction = np.zeros(3)
    feedback.curr_pos = np.array([0.4, 0.0, 0.3])
    feedback.get_transform = lambda *_args: np.eye(4)
    feedback.get_logger = lambda: SimpleNamespace(info=lambda *_a: None, warning=lambda *_a: None)
    feedback.bridge = SimpleNamespace(cv2_to_imgmsg=lambda *_a, **_kw: None)
    feedback.current_template_pub = SimpleNamespace(publish=lambda *_a: None)
    feedback.marker_pub = SimpleNamespace(publish=lambda *_a: None)
    feedback.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time())
    )
    return feedback


def _run_window(feedback):
    for _ in range(feedback.sift_window):
        feedback.sift_matching()


def test_correction_scales_with_pixel_error():
    small = _feedback_with_shift(4)
    large = _feedback_with_shift(16)

    _run_window(small)
    _run_window(large)

    # sign follows the image shift, magnitude follows the pixel error (clamped)
    assert small.camera_correction[0] > 0
    assert abs(large.camera_correction[0]) > abs(small.camera_correction[0])
    assert abs(large.camera_correction[0]) <= large.max_correction_step
    np.testing.assert_allclose(small.camera_correction[0], 0.001 * 4, atol=5e-4)

    # an uncorrected image error keeps the timestep held
    assert large.sift_converged is False


def test_no_correction_before_window_is_full():
    feedback = _feedback_with_shift(16)

    for _ in range(feedback.sift_window - 1):
        feedback.sift_matching()

    np.testing.assert_array_equal(feedback.camera_correction, np.zeros(3))


def test_moving_arm_gathers_instead_of_correcting():
    feedback = _feedback_with_shift(16)
    positions = iter([np.array([0.4, 0.0, 0.3]) + np.array([0.01 * i, 0.0, 0.0]) for i in range(20)])
    feedback.curr_pos = np.array([0.4, 0.0, 0.3])

    for _ in range(2 * feedback.sift_window):
        feedback.curr_pos = next(positions)   # arm still travelling, 10 mm between measurements
        feedback.sift_matching()

    np.testing.assert_array_equal(feedback.camera_correction, np.zeros(3))


def test_matched_image_reports_converged():
    feedback = _feedback_with_shift(0)
    feedback.sift_converged = False

    _run_window(feedback)

    assert feedback.sift_converged is True
    np.testing.assert_allclose(feedback.camera_correction, np.zeros(3), atol=1e-9)
