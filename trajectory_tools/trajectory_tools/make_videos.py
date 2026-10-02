#!/usr/bin/env python3
"""
Generate an mp4 video for each .npz trajectory file in a folder.
Output videos are saved alongside the .npz files with the same stem name.

Usage:
    ros2 run trajectory_tools make_videos [trajectories_dir] [--fps FPS] [--no-overlay] [--jobs N]

Defaults:
    trajectories_dir  = <trajectory_data>/trajectories
    fps               = 30
    jobs              = number of CPU cores
"""
import argparse
import multiprocessing
import sys
from pathlib import Path

import cv2
import numpy as np

import trajectory_data


def overlay_hud(frame_bgr: np.ndarray, frame_idx: int, total: int,
                grip: float, flags: dict[str, int]) -> np.ndarray:
    """Burn a small status bar onto the frame (in-place copy)."""
    img = frame_bgr.copy()
    h, w = img.shape[:2]

    # semi-transparent dark bar at the bottom
    bar_h = 18
    overlay = img.copy()
    cv2.rectangle(overlay, (0, h - bar_h), (w, h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, img, 0.45, 0, img)

    active = [k for k, v in flags.items() if v]
    flag_str = " ".join(active) if active else ""
    text = f"{frame_idx+1}/{total}  grip={grip:.2f}  {flag_str}"
    cv2.putText(img, text, (3, h - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 230, 255), 1, cv2.LINE_AA)
    return img


def npz_to_mp4(npz_path: Path, fps: int, add_overlay: bool) -> str:
    out_path = npz_path.with_suffix(".mp4")
    if out_path.exists():
        return f"skip  {out_path.name}"

    try:
        data = np.load(npz_path, allow_pickle=True)
    except Exception as e:
        return f"ERROR loading {npz_path.name}: {e}"

    if "img" not in data:
        return f"skip  {npz_path.name}  (no 'img' key)"

    imgs = data["img"]          # (N, H, W) uint8 grayscale
    N, H, W = imgs.shape

    def get(key, dtype=float):
        if key not in data:
            return np.zeros(N, dtype=dtype)
        arr = data[key].flatten()
        if len(arr) < N:
            arr = np.pad(arr, (0, N - len(arr)))
        return arr[:N].astype(dtype)

    grip_arr = get("grip", float)
    img_fb   = get("img_feedback_flag", int)
    spiral   = get("spiral_flag", int)
    risk     = get("risk_flag", int)
    safe     = get("safe_flag", int)
    novelty  = get("novelty_flag", int)

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(out_path), fourcc, fps, (W, H))
    if not writer.isOpened():
        return f"ERROR could not open VideoWriter for {out_path.name}"

    for i in range(N):
        frame_bgr = cv2.cvtColor(imgs[i], cv2.COLOR_GRAY2BGR)
        if add_overlay:
            flags = {
                "img_fb": int(img_fb[i]),
                "spiral": int(spiral[i]),
                "risk":   int(risk[i]),
                "safe":   int(safe[i]),
                "novel":  int(novelty[i]),
            }
            frame_bgr = overlay_hud(frame_bgr, i, N, float(grip_arr[i]), flags)
        writer.write(frame_bgr)

    writer.release()
    return f"done  {out_path.name}  ({N} frames)"


def worker(args):
    return npz_to_mp4(*args)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("trajectories_dir", nargs="?",
                        default=f"{trajectory_data.package_path}/trajectories",
                        help="Folder containing .npz trajectory files")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--no-overlay", action="store_true",
                        help="Omit the HUD overlay (frame counter, grip, flags)")
    parser.add_argument("--jobs", type=int, default=multiprocessing.cpu_count(),
                        help="Parallel worker processes (default: all cores)")
    args = parser.parse_args()

    traj_dir = Path(args.trajectories_dir)
    if not traj_dir.is_dir():
        sys.exit(f"Directory not found: {traj_dir}")

    npz_files = sorted(traj_dir.glob("*.npz"))
    if not npz_files:
        sys.exit(f"No .npz files found in {traj_dir}")

    print(f"Found {len(npz_files)} trajectories in {traj_dir}")
    print(f"FPS={args.fps}  overlay={'off' if args.no_overlay else 'on'}  jobs={args.jobs}\n")

    add_overlay = not args.no_overlay
    tasks = [(f, args.fps, add_overlay) for f in npz_files]

    if args.jobs == 1:
        for task in tasks:
            print(worker(task))
    else:
        with multiprocessing.Pool(args.jobs) as pool:
            for result in pool.imap_unordered(worker, tasks):
                print(result)

    print("\nDone.")


if __name__ == "__main__":
    main()
