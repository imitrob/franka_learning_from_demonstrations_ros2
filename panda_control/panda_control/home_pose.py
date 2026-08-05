"""Canonical Cartesian home pose for the Panda end effector.

Keep this module dependency-free: launch files and packages which need to
reason about the parked arm should be able to import the pose without starting
the robot-control stack.
"""

from typing import NamedTuple


class CartesianPose(NamedTuple):
    position: tuple[float, float, float]
    orientation_wxyz: tuple[float, float, float, float]


# Position is expressed in panda_link0 (metres); orientation is scalar-first.
HOME_POSE = CartesianPose(
    position=(0.4, 0.0, 0.4),
    orientation_wxyz=(0.0, 1.0, 0.0, 0.0),
)
