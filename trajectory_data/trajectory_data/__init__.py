
import os
path = os.path.dirname(os.path.realpath(__file__))
# TRAJECTORY_DATA_PATH points to the package source dir (holds trajectories/);
# without it the ROS2 build dir is used (see franka_hri README)
package_path = os.path.expanduser(os.environ.get("TRAJECTORY_DATA_PATH", "/".join(path.split("/")[:-1])))

from trajectory_data.skill_part import SkillPart  # noqa: E402

__all__ = ["SkillPart"]
