import numpy as np
import pytest

from trajectory_data.skill_part import SkillPart


def _archive(path, length=3, **changes):
    arrays = {
        "traj": np.zeros((3, length)),
        "ori": np.zeros((4, length)),
        "grip": np.zeros((1, length)),
        "img": np.zeros((length, 2, 2)),
        "img_feedback_flag": np.zeros((1, length)),
        "spiral_flag": np.zeros((1, length)),
        "risk_flag": np.zeros((1, length)),  # extra RALfD data is allowed
    }
    arrays.update(changes)
    np.savez(path, **arrays)


def test_skill_part_parses_task_part_and_object():
    part = SkillPart("put2__bowl_trial_3.npz")

    assert (part.action, part.part, part.object, part.trial) == (
        "put", "2", "bowl", 3,
    )
    assert part.to_str() == "put2__bowl_trial_3"


def test_validate_archive_accepts_an_lfd_archive(tmp_path):
    _archive(tmp_path / "pick__cube.npz")

    SkillPart("pick__cube").validate_archive(tmp_path)


@pytest.mark.parametrize("changes,reason", [
    ({"ori": np.zeros((4, 2))}, "ori must have shape"),
    ({"traj": np.zeros((3, 0)), "ori": np.zeros((4, 0)),
      "grip": np.zeros((1, 0)), "img": np.zeros((0, 2, 2)),
      "img_feedback_flag": np.zeros((1, 0)),
      "spiral_flag": np.zeros((1, 0))}, "traj must have shape"),
])
def test_validate_archive_rejects_bad_shapes(tmp_path, changes, reason):
    _archive(tmp_path / "bad.npz", **changes)

    with pytest.raises(ValueError, match=reason):
        SkillPart("bad").validate_archive(tmp_path)


def test_validate_archive_rejects_missing_array(tmp_path):
    np.savez(tmp_path / "bad.npz", traj=np.zeros((3, 1)))

    with pytest.raises(ValueError, match="missing arrays"):
        SkillPart("bad").validate_archive(tmp_path)


def test_validate_archive_rejects_corrupt_and_missing_files(tmp_path):
    corrupt = tmp_path / "corrupt.npz"
    corrupt.write_text("not a numpy archive")

    with pytest.raises(ValueError, match="cannot read skill part"):
        SkillPart("corrupt").validate_archive(tmp_path)
    with pytest.raises(FileNotFoundError):
        SkillPart("missing").validate_archive(tmp_path)
