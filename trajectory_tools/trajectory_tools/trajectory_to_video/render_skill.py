"""Replay a recorded skill (.npz) on a simulated Panda in Swift and save an mp4.

The npz holds end-effector poses, not joints: `traj` (3, N) position,
`ori` (4, N) quaternion w x y z (lfd's recorded_ori_wxyz), `grip` (1, N) finger width.
Each frame is solved with IK, warm-started from the previous one.

    ros2 run trajectory_tools render_skill abhi_kin_peg_pick -o peg_pick.mp4
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as sg
from spatialmath import SE3, UnitQuaternion

import trajectory_data
from trajectory_tools.trajectory_to_video.check_mesh import TASKBOARD
from trajectory_tools.trajectory_to_video.swift_render import SwiftCamera

TRAJECTORIES = Path(trajectory_data.package_path) / "trajectories"


def load_skill(path):
    with np.load(path) as d:
        return d["traj"], d["ori"], d["grip"][0]


def solve_joints(robot, traj, ori):
    """One joint vector per waypoint; count of waypoints IK did not reach."""
    q, qs, failed = robot.qr, [], 0
    for p, (w, x, y, z) in zip(traj.T, ori.T):
        ee = SE3(*p) * UnitQuaternion(w, [x, y, z]).SE3()
        # rtb's Panda already solves for Franka's EE frame (O_T_EE): its end
        # link is panda_hand with the 0.1034 m gripper tool, not the flange.
        sol, ok, *_ = robot.ik_LM(ee.A, q0=q)
        failed += not ok
        q = sol if ok else q  # ponytail: hold the last reachable pose on failure
        qs.append(q)
    return np.array(qs), failed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("skill", help="npz path, or a name in the trajectories folder")
    ap.add_argument("-o", "--out", default=None,
                    help="mp4 path, or a folder to write <skill>.mp4 into (default: here)")
    ap.add_argument("--mesh", default=str(TASKBOARD))
    ap.add_argument("--mesh-scale", type=float, default=0.001, help="mesh units to metres")
    # ponytail: guessed board placement; z lifts the mesh's -79.5 mm bottom onto the table
    ap.add_argument("--mesh-xyz", type=float, nargs=3, default=[0.5, -0.15, 0.0795])
    ap.add_argument("--mesh-yaw", type=float, default=270.0, help="degrees")
    ap.add_argument("--fps", type=int, default=30, help="recording rate of the npz")
    ap.add_argument("--stride", type=int, default=1, help="render every n-th waypoint")
    ap.add_argument("--eye", type=float, nargs=3, default=[0.92, 0.5, 0.51],
                    help="camera position, robot base frame")
    ap.add_argument("--center", type=float, nargs=2, default=[-0.1, -0.1],
                    help="table point (x y) the camera looks at, robot base frame")
    ap.add_argument("--visible", action="store_true")
    ap.add_argument("--cpu", action="store_true", help="software GL, when no GPU is usable")
    args = ap.parse_args()

    path = Path(args.skill)
    if not path.exists():
        path = TRAJECTORIES / path.with_suffix(".npz").name
    out = Path(args.out or ".")
    if out.is_dir():
        out = out / f"{path.stem}.mp4"

    robot = rtb.models.Panda()
    traj, ori, grip = load_skill(path)
    idx = np.arange(0, traj.shape[1], args.stride)
    joints, failed = solve_joints(robot, traj[:, idx], ori[:, idx])
    print(f"{path.name}: {len(idx)} frames, IK failed on {failed}")
    # The camera only looks at the scene origin (SwiftCamera.look), so move the
    # scene to put `center` there. Only in x y: Swift's floor at z=0 is opaque. After IK: ik_LM solves relative to robot.base.
    shift = SE3(-args.center[0], -args.center[1], 0)
    robot.base = shift

    with SwiftCamera(visible=args.visible, gpu=not args.cpu) as cam:
        cam.env.add(robot)
        if args.mesh:
            cam.env.add(sg.Mesh(str(Path(args.mesh).resolve()), scale=[args.mesh_scale] * 3,
                                color=(0.35, 0.38, 0.42, 1.0),
                                pose=shift * SE3(*args.mesh_xyz) * SE3.Rz(args.mesh_yaw, unit="deg")))
        cam.look(np.subtract(args.eye, [*args.center, 0]))
        video = None
        for q, width in zip(joints, grip[idx]):
            robot.q = q
            robot.grippers[0].q = [width / 2, width / 2]
            cam.env.step(0)
            frame = cam.frame()
            if video is None:
                h, w = frame.shape[:2]
                video = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"),
                                        args.fps / args.stride, (w, h))
            video.write(frame)
        video.release()
    print(f"saved {out}")


if __name__ == "__main__":
    main()
