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
