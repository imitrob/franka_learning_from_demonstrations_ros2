import cv2
import inspect
import numpy as np
import pytest
import yaml
from geometry_msgs.msg import PoseStamped

import object_localization
from object_localization.template_recorder_server import TemplateRecorderServer
import object_localization.template_recorder_server as recorder_module
import object_localization.record_template as template_client
import object_localization.gui.gui_template as template_gui


def test_template_save_is_complete_and_rejects_implicit_overwrite(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(object_localization, "package_path", str(tmp_path))
    server = object.__new__(TemplateRecorderServer)
    color = np.full((10, 12, 3), 127, dtype=np.uint8)
    depth = np.full((10, 12), 500, dtype=np.uint16)
    pose = PoseStamped()
    pose.pose.position.x = 0.4
    pose.pose.orientation.w = 1.0

    saved = server._save(
        "cube", color, depth, pose, (2, 8, 1, 7), overwrite=False
    )
    with open(f"{saved}/params.yaml") as stream:
        params = yaml.safe_load(stream)
    assert params["crop"] == [2, 8, 1, 7]
    assert params["depth"] == 500.0
    assert cv2.imread(f"{saved}/template.png").shape[:2] == (6, 6)

    with pytest.raises(FileExistsError):
        server._save("cube", color, depth, pose, (2, 8, 1, 7), False)


def test_template_crop_maps_color_coordinates_to_depth_resolution(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(object_localization, "package_path", str(tmp_path))
    server = object.__new__(TemplateRecorderServer)
    color = np.full((10, 12, 3), 127, dtype=np.uint8)
    depth = np.full((5, 6), 700, dtype=np.uint16)
    pose = PoseStamped()

    saved = server._save(
        "mismatched_resolution", color, depth, pose,
        (7, 11, 4, 9), overwrite=False,
    )

    with open(f"{saved}/params.yaml") as stream:
        assert yaml.safe_load(stream)["depth"] == 700.0


def test_template_workflow_never_constructs_panda():
    source = "\n".join(inspect.getsource(module) for module in (
        recorder_module, template_client, template_gui,
    ))
    assert "panda_control" not in source
    assert "SpinPandaNode" not in source
