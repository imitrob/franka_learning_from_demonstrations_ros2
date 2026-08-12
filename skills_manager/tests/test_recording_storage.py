import numpy as np
import pytest

import trajectory_data
from skills_manager.lfd import LfD


def test_skill_save_is_validated_atomic_and_requires_explicit_overwrite(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(trajectory_data, "package_path", str(tmp_path))
    lfd = object.__new__(LfD)
    length = 3
    lfd.recorded_traj = np.zeros((3, length))
    lfd.recorded_ori_wxyz = np.zeros((4, length))
    lfd.recorded_gripper = np.zeros((1, length))
    lfd.recorded_img = np.zeros((length, 2, 2), dtype=np.uint8)
    lfd.recorded_img_feedback_flag = np.zeros((1, length))
    lfd.recorded_spiral_flag = np.zeros((1, length))
    lfd.final_transform = None

    assert lfd.save("pick__cube", overwrite=False)
    with pytest.raises(FileExistsError):
        lfd.save("pick__cube", overwrite=False)
    assert list((tmp_path / "trajectories").glob("*.npz")) == [
        tmp_path / "trajectories" / "pick__cube.npz"
    ]
