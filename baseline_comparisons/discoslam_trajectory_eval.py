#!/usr/bin/env python3
"""
Zero-Rotation & Zero-Translation Normalization with Frame Presets

Adds a default frame preset so you don't need to pass --neg_x --neg_y every time.
- DEFAULT_CSV2_PRESET = "neg_xy" (flip X and Y signs for CSV #2)  <-- change here if needed
- You can still override via CLI: --frame_preset none|neg_xy|swap_xy|neg_x|neg_y|swap_xy_negx|swap_xy_negy
- Individual overrides (--swap_xy, --neg_x, --neg_y) apply AFTER the preset.

Usage:
  python trajectory_eval_zero_rot_preset.py --csv1 ref.csv --csv2 est.csv --outdir results_zero_rot
"""
from __future__ import annotations

import argparse
import os
import sys
from typing import Tuple

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation, Slerp
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---- Configure your default here ----
DEFAULT_CSV2_PRESET = "neg_xy"  # options: none, neg_xy, swap_xy, neg_x, neg_y, swap_xy_negx, swap_xy_negy


def _read_pose_csv(path: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    df = pd.read_csv(path, comment="#", header=0)
    cols = {c.lower().strip(): c for c in df.columns}
    time_col = None
    for k in ("timestamp", "stamp", "time", "t"):
        if k in cols: time_col = cols[k]; break
    if time_col is None:
        raise ValueError(f"No time column ('timestamp'/'stamp'/'time'/'t'): {path}")
    for k in ("x","y","z","qx","qy","qz","qw"):
        if k not in cols: raise ValueError(f"Missing column: {k}")
    t = df[time_col].to_numpy(dtype=np.float64)
    p = df[[cols["x"], cols["y"], cols["z"]]].to_numpy(dtype=np.float64)
    q = df[[cols["qx"], cols["qy"], cols["qz"], cols["qw"]]].to_numpy(dtype=np.float64)
    idx = np.argsort(t); t,p,q = t[idx], p[idx], q[idx]
    q = q / np.clip(np.linalg.norm(q, axis=1, keepdims=True), 1e-12, None)
    # sign continuity
    for i in range(1, len(q)):
        if np.dot(q[i-1], q[i]) < 0: q[i] = -q[i]
    return t, p, q


def interpolate_poses(t_src, p_src, q_src, t_query):
    if len(t_src) < 2: raise ValueError("Need >=2 source samples.")
    t_min, t_max = t_src[0], t_src[-1]
    if np.any(t_query < t_min) or np.any(t_query > t_max):
        sys.stderr.write("Warning: some query timestamps are outside source range; clipping.\\n")
    tq = np.clip(t_query, t_min, t_max)
    p_q = np.column_stack([np.interp(tq, t_src, p_src[:,i]) for i in range(3)])
    slerp = Slerp(t_src, Rotation.from_quat(q_src))
    q_q = slerp(tq).as_quat()
    for i in range(1, len(q_q)):
        if np.dot(q_q[i-1], q_q[i]) < 0: q_q[i] = -q_q[i]
    return p_q, q_q


def poses_to_T(p, q):
    R = Rotation.from_quat(q).as_matrix()
    N = p.shape[0]
    T = np.repeat(np.eye(4)[None, ...], N, axis=0)
    T[:, :3, :3] = R
    T[:, :3, 3] = p
    return T


def invert_SE3(T):
    single = False
    if T.ndim == 2: T = T[None, ...]; single = True
    R = T[:, :3, :3]; t = T[:, :3, 3:4]
    RT = np.transpose(R, (0,2,1))
    tinv = -RT @ t
    Tout = np.repeat(np.eye(4)[None, ...], T.shape[0], axis=0)
    Tout[:, :3, :3] = RT
    Tout[:, :3, 3] = tinv[:, :, 0]
    return Tout[0] if single else Tout


def left_multiply(A, B):
    if A.ndim == 2: A = A[None, ...]
    if B.ndim == 2: B = B[None, ...]
    if A.shape[0] == 1 and B.shape[0] > 1: A = np.repeat(A, B.shape[0], axis=0)
    return A @ B


def compute_ate(p_est, p_ref):
    d = np.linalg.norm(p_est - p_ref, axis=1)
    return {
        "rmse": float(np.sqrt(np.mean(d**2))),
        "mean": float(np.mean(d)),
        "median": float(np.median(d)),
        "std": float(np.std(d)),
        "min": float(np.min(d)),
        "max": float(np.max(d)),
        "num": int(len(d)),
    }


def SE3_log(Rt):
    t = Rt[:3, 3]
    trans = float(np.linalg.norm(t))
    R = Rt[:3, :3]
    ang = float(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))
    return trans, ang


def compute_rpe(T_est, T_ref, delta=1):
    N = T_est.shape[0]
    if N != T_ref.shape[0]: raise ValueError("Length mismatch for RPE.")
    if delta < 1 or delta >= N: raise ValueError("delta must be in [1, N-1].")
    trans_err, rot_err = [], []
    for i in range(N - delta):
        Te = invert_SE3(T_est[i]) @ T_est[i+delta]
        Tr = invert_SE3(T_ref[i]) @ T_ref[i+delta]
        E = invert_SE3(Tr) @ Te
        t_norm, a = SE3_log(E)
        trans_err.append(t_norm); rot_err.append(a)
    trans_err = np.asarray(trans_err); rot_err = np.asarray(rot_err)
    return {
        "rmse_trans": float(np.sqrt(np.mean(trans_err**2))),
        "rmse_rot_deg": float(np.sqrt(np.mean(np.degrees(rot_err)**2))),
        "mean_trans": float(np.mean(trans_err)),
        "mean_rot_deg": float(np.mean(np.degrees(rot_err))),
        "num": int(len(trans_err)),
    }


def apply_preset(p: np.ndarray, preset: str) -> np.ndarray:
    p = p.copy()
    preset = (preset or "none").lower()
    if preset == "none":
        return p
    elif preset == "neg_xy":
        p[:, 0] = -p[:, 0]
        p[:, 1] = -p[:, 1]
    elif preset == "swap_xy":
        p[:, [0,1]] = p[:, [1,0]]
    elif preset == "neg_x":
        p[:, 0] = -p[:, 0]
    elif preset == "neg_y":
        p[:, 1] = -p[:, 1]
    elif preset == "swap_xy_negx":
        p[:, [0,1]] = p[:, [1,0]]
        p[:, 0] = -p[:, 0]
    elif preset == "swap_xy_negy":
        p[:, [0,1]] = p[:, [1,0]]
        p[:, 1] = -p[:, 1]
    else:
        raise ValueError(f"Unknown frame preset: {preset}")
    return p


def save_csv(path, t, p, q):
    df = pd.DataFrame({
        "timestamp": t,
        "x": p[:,0], "y": p[:,1], "z": p[:,2],
        "qx": q[:,0], "qy": q[:,1], "qz": q[:,2], "qw": q[:,3],
    })
    os.makedirs(os.path.dirname(path), exist_ok=True)
    df.to_csv(path, index=False)


def plot_xy(path, p_est, p_ref):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    plt.figure()
    plt.plot(p_est[:,0], p_est[:,1], label="CSV #2 (normed)", alpha=0.9)
    plt.plot(p_ref[:,0], p_ref[:,1], "--", label="CSV #1 interp (normed)", alpha=0.9)
    plt.xlabel("x (m)"); plt.ylabel("y (m)")
    plt.legend(); plt.axis("equal"); plt.tight_layout()
    plt.savefig(path, dpi=150); plt.close()


def evalute_results(env, robot_num, exp_num):
    rpe_delta = 1
    data_path = os.path.join("./traj_eval_data", f"{env}")
    outdir = os.path.join(data_path, f"results_{env}{exp_num}")

    csv1 = os.path.join(data_path, f"true_paths_{env}", f"{env}_robot{robot_num}_ref.csv")
    csv2 = os.path.join(data_path, f"final_paths_{env}{exp_num}", f"robot{robot_num}_path.csv")

    t1, p1, q1 = _read_pose_csv(csv1)
    t2, p2, q2 = _read_pose_csv(csv2)

    # Apply preset to CSV #2 (positions only), then optional manual overrides
    p2_adj = apply_preset(p2, DEFAULT_CSV2_PRESET)

    # Interpolate ref onto t2
    p1_q, q1_q = interpolate_poses(t1, p1, q1, t2)

    # Build SE(3)
    T_ref = poses_to_T(p1_q, q1_q)
    T_est = poses_to_T(p2_adj, q2)

    # Normalize BOTH to identity (zero translation and rotation at start)
    T_ref0_inv = invert_SE3(T_ref[0])
    T_est0_inv = invert_SE3(T_est[0])
    T_ref_n = left_multiply(T_ref0_inv, T_ref)
    T_est_n = left_multiply(T_est0_inv, T_est)

    # Extract normalized p, q
    p_ref_n = T_ref_n[:, :3, 3]
    q_ref_n = Rotation.from_matrix(T_ref_n[:, :3, :3]).as_quat()
    p_est_n = T_est_n[:, :3, 3]
    q_est_n = Rotation.from_matrix(T_est_n[:, :3, :3]).as_quat()

    # Save
    save_csv(os.path.join(outdir, f"robot{robot_num}_csv1_interpolated_normed.csv"), t2, p_ref_n, q_ref_n)
    save_csv(os.path.join(outdir, f"robot{robot_num}_csv2_normed.csv"), t2, p_est_n, q_est_n)

    # Metrics
    dists = compute_ate(p_est_n, p_ref_n)
    rpe = compute_rpe(T_est_n, T_ref_n, delta=rpe_delta)

    # Plot
    plot_xy(os.path.join(outdir, f"robot{robot_num}_trajectory_xy.png"), p_est_n, p_ref_n)

    return dists["rmse"], rpe["rmse_trans"]

    # # Print
    # print("=== Outputs ===")
    # print("CSV1 (interp, normed):", os.path.join(outdir, "csv1_interpolated_normed.csv"))
    # print("CSV2 (normed):        ", os.path.join(outdir, "csv2_normed.csv"))
    # print("XY plot:", os.path.join(outdir, "trajectory_xy_zero_rot.png"))
    # print("\n=== ATE (position, normed) [m] ===")
    # for k in ["rmse", "mean", "median", "std", "min", "max", "num"]:
    #     print(f"{k:>8}: {dists[k]:.6f}" if k != "num" else f"{k:>8}: {dists[k]}")
    # print(f"\n=== RPE (Δ={rpe_delta}, normed) ===")
    # print("Trans RMSE [m]:   {:.6f}".format(rpe["rmse_trans"]))
    # print("Rot RMSE   [deg]: {:.6f}".format(rpe["rmse_rot_deg"]))
    # print("Trans mean [m]:   {:.6f}".format(rpe["mean_trans"]))
    # print("Rot mean   [deg]: {:.6f}".format(rpe["mean_rot_deg"]))
    # print("Num pairs:        {}".format(rpe["num"]))
        
def main():
    robots = [1,2,3,4]
    envs = ["kittredge_loop", "main_campus"]
    num_experiments = 3

    for env in envs:
        print(f"env: {env}")
        for robot in robots:
            print(f"   robot: {robot}")
            robot_rsme_list = []
            robot_rpe_list = []
            for exp_num in range(1,num_experiments+1):
                # print(f"       exp_num: {exp_num}")
                rsme, rpe = evalute_results(env, robot, exp_num)
                robot_rsme_list.append(rsme)
                robot_rpe_list.append(rpe)
            ave_ate = np.average(robot_rsme_list)
            ave_rpe = np.average(robot_rpe_list)

            print(f"        - ave ate: {ave_ate}, ave_rpe: {ave_rpe}")

if __name__ == "__main__":
    main()
