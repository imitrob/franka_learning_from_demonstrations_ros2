"""Does a custom mesh render in Swift? Renders the scene without and with the mesh
and asserts enough pixels changed. Exit 0 = mesh visible.

    ros2 run trajectory_tools check_mesh [mesh.stl] [--scale 0.001] [--out check_mesh.png]
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
import spatialgeometry as sg
from spatialmath import SE3

from trajectory_tools.trajectory_to_video.swift_render import SwiftCamera

TASKBOARD = Path(__file__).resolve().parent / "taskboard.stl"
MIN_CHANGED = 0.002  # ponytail: 0.2% of pixels; a missing mesh changes none


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mesh", nargs="?", default=str(TASKBOARD))
    ap.add_argument("--scale", type=float, default=0.001, help="mesh units to metres")
    ap.add_argument("--out", default="check_mesh.png")
    ap.add_argument("--visible", action="store_true")
    args = ap.parse_args()

    with SwiftCamera(visible=args.visible) as cam:
        cam.look([0.5, 0.5, 0.4])
        cam.env.step(0)
        before = cam.frame()
        mesh = sg.Mesh(str(Path(args.mesh).resolve()), scale=[args.scale] * 3,
                       color=(0.2, 0.4, 0.9, 1.0), pose=SE3())
        cam.env.add(mesh)
        cam.env.step(0)
        after = cam.frame()

    cv2.imwrite(args.out, np.hstack([before, after]))
    changed = np.mean(np.abs(after.astype(int) - before.astype(int)).sum(2) > 30)
    print(f"{changed:.1%} of pixels changed; before|after saved to {args.out}")
    assert changed > MIN_CHANGED, "mesh did not render"
    print("PASS")


if __name__ == "__main__":
    main()
