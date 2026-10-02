"""A recorded trajectory part and the filename convention that identifies it.

    <action>[<part>][__<object>]
        [_branch_from_<parent>_at_<offset>][_trial_<n>].npz

Examples:

    pick__cube                     one-part ``pick cube`` task
    put1__cube / put2__bowl        two parts of ``put cube to bowl``
    pick__cube_trial_3             an execution trial of the same part
    pick__cube_branch_from_0_at_58 a branch of the same part
"""
import glob
import os

import numpy as np


def trajectories_directory() -> str:
    """Return the configured trajectory archive directory."""
    import trajectory_data
    return os.path.join(trajectory_data.package_path, "trajectories")


class SkillPart:
    """One recorded ``.npz`` trajectory, split into its filename metadata."""

    SEP = "__"
    BRANCH_FROM = "_branch_from_"
    BRANCH_AT = "_branch_at_"
    TRIAL = "_trial_"

    REQUIRED_ARRAYS = (
        "traj",
        "ori",
        "grip",
        "img",
        "img_feedback_flag",
        "spiral_flag",
    )

    def __init__(self, filename: str):
        self.filename = os.path.basename(filename)
        if not self.filename.endswith(".npz"):
            self.filename += ".npz"
        self.name = self.filename.removesuffix(".npz")

        rest = self.name
        rest, self.trial = _split_int(rest, self.TRIAL, default=-1)

        self.parent_offset, self.offset = 0, 0
        head, sep, branch = rest.partition(self.BRANCH_FROM)
        if sep:
            parent, _, offset = branch.partition("_at_")
            self.parent_offset, self.offset = _int(parent), _int(offset)
            rest = head
        else:
            rest, self.offset = _split_int(rest, self.BRANCH_AT, default=0)

        self.base = rest
        action_part, _, self.object = rest.partition(self.SEP)
        if len(action_part) > 1 and action_part[-1].isdigit():
            self.action, self.part = action_part[:-1], action_part[-1]
        else:
            self.action, self.part = action_part, None

    @property
    def action_part(self) -> str:
        """The action as it appears in the filename: ``put1`` or ``pick``."""
        return f"{self.action}{self.part or ''}"

    @property
    def is_variant(self) -> bool:
        """Whether this is a trial or branch rather than a base recording."""
        return self.trial != -1 or self.offset != 0

    @property
    def is_demo(self) -> bool:
        return self.trial == -1

    def to_str(self) -> str:
        """Render the parsed filename without its extension."""
        name = self.action_part + (f"{self.SEP}{self.object}" if self.object else "")
        if self.offset:
            name += f"{self.BRANCH_FROM}{self.parent_offset}_at_{self.offset}"
        if self.trial != -1:
            name += f"{self.TRIAL}{self.trial}"
        return name

    def archive_path(self, directory: str = None) -> str:
        return os.path.join(directory or trajectories_directory(), self.filename)

    def validate_archive(self, directory: str = None) -> None:
        """Raise when the archive cannot be played by ordinary ``LfD``."""
        path = self.archive_path(directory)
        try:
            archive = np.load(path, allow_pickle=False)
        except FileNotFoundError:
            raise
        except Exception as exc:
            raise ValueError(f"cannot read skill part {self.filename}: {exc}") from exc

        try:
            with archive:
                missing = [key for key in self.REQUIRED_ARRAYS if key not in archive]
                if missing:
                    raise ValueError(
                        f"invalid skill part {self.filename}: "
                        f"missing arrays: {', '.join(missing)}"
                    )
                arrays = {key: archive[key] for key in self.REQUIRED_ARRAYS}
        except ValueError as exc:
            if str(exc).startswith("invalid skill part"):
                raise
            raise ValueError(f"cannot read skill part {self.filename}: {exc}") from exc
        except Exception as exc:
            raise ValueError(f"cannot read skill part {self.filename}: {exc}") from exc

        traj = arrays["traj"]
        if traj.ndim != 2 or traj.shape[0] != 3 or traj.shape[1] == 0:
            raise ValueError(
                f"invalid skill part {self.filename}: traj must have shape (3, N), N > 0"
            )
        length = traj.shape[1]
        expected = {
            "ori": (4, length),
            "grip": (1, length),
            "img_feedback_flag": (1, length),
            "spiral_flag": (1, length),
        }
        for key, shape in expected.items():
            if arrays[key].shape != shape:
                raise ValueError(
                    f"invalid skill part {self.filename}: {key} must have shape {shape}, "
                    f"got {arrays[key].shape}"
                )
        if arrays["img"].ndim < 1 or arrays["img"].shape[0] != length:
            raise ValueError(
                f"invalid skill part {self.filename}: img must contain {length} frames"
            )

    def __repr__(self):
        return (f"SkillPart({self.name!r}: action={self.action!r} part={self.part!r} "
                f"object={self.object!r} offset={self.offset} trial={self.trial})")

    @classmethod
    def scan(cls, directory: str = None, variants: bool = False) -> list:
        """Return recorded parts, excluding trials and branches by default."""
        root = directory or trajectories_directory()
        found = [cls(path) for path in sorted(glob.glob(os.path.join(root, "*.npz")))]
        return found if variants else [part for part in found if not part.is_variant]

    @classmethod
    def for_combination(cls, parts, action: str, obj: str = None) -> list:
        """Return parts matching an action or action-object combination."""
        return [part for part in parts if part.action == action
                and (obj is None or part.object == obj)]


def _int(text: str, default: int = 0) -> int:
    try:
        return int(text)
    except ValueError:
        return default


def _split_int(name: str, marker: str, default: int):
    head, sep, tail = name.rpartition(marker)
    if not sep or not tail.isdigit():
        return name, default
    return head, int(tail)
