#!/usr/bin/env python3
import os
import sys
import csv
import numpy as np
import open3d as o3d


import numpy as np
from typing import Tuple, List, Dict

import numpy as np


import numpy as np

def symmetric_overlap_stats(A_pos, B_pos, radius, step=0.25, use_xy_only=True):
    # Directed A→B
    A_ov, A_tot, A_cov, _ = overlap_length_spatial(A_pos, B_pos, radius, step, use_xy_only)
    # Directed B→A
    B_ov, B_tot, B_cov, _ = overlap_length_spatial(B_pos, A_pos, radius, step, use_xy_only)

    sym_len = 0.5 * (A_ov + B_ov)                                # meters
    dice_pct = 0.0 if (A_tot + B_tot) == 0 else (A_ov + B_ov) / (A_tot + B_tot) * 100.0
    avg_cov_pct = 0.5 * (A_cov + B_cov)                          # mean of directed %

    return {
        "A_ov_m": A_ov, "A_tot_m": A_tot, "A_cov_pct": A_cov,
        "B_ov_m": B_ov, "B_tot_m": B_tot, "B_cov_pct": B_cov,
        "sym_len_m": sym_len, "dice_pct": dice_pct, "avg_cov_pct": avg_cov_pct,
    }

def _arc_lengths(pts: np.ndarray) -> np.ndarray:
    segs = np.diff(pts, axis=0)
    seglen = np.linalg.norm(segs, axis=1)
    s = np.concatenate([[0.0], np.cumsum(seglen)])
    return s

def _interp_along_polyline(pts: np.ndarray, s_query: np.ndarray) -> np.ndarray:
    """Linear interpolation along cumulative arc-length."""
    s = _arc_lengths(pts)
    if s[-1] == 0:
        return np.repeat(pts[:1], len(s_query), axis=0)
    s_query = np.clip(s_query, 0.0, s[-1])
    idx = np.searchsorted(s, s_query, side="right").clip(1, len(s)-1)
    s0, s1 = s[idx-1], s[idx]
    p0, p1 = pts[idx-1], pts[idx]
    t = np.divide(s_query - s0, s1 - s0, out=np.zeros_like(s_query), where=(s1 > s0))
    return (1 - t)[:, None] * p0 + t[:, None] * p1

def resample_polyline(pts: np.ndarray, step: float) -> np.ndarray:
    """
    Resample a polyline to ~uniform spacing 'step' (keeps endpoints).
    Returns an array of shape (M, D).
    """
    pts = np.asarray(pts, float)
    if pts.shape[0] == 0:
        return pts
    if pts.shape[0] == 1:
        return pts.copy()
    s = _arc_lengths(pts)
    L = s[-1]
    if L == 0:
        return pts[:1].copy()
    # include both ends
    n = max(1, int(np.floor(L / step)))
    s_samples = np.linspace(0.0, L, n + 1)
    return _interp_along_polyline(pts, s_samples)

def overlap_length_spatial(
    source_pos: np.ndarray,
    target_pos: np.ndarray,
    radius: float,
    step: float = 0.25,
    use_xy_only: bool = True,
):
    """
    Time-agnostic overlap: length of source path that comes within `radius` of target path.
    Uses resampling + KDTree and checks endpoints + midpoint of each resampled segment.
    Returns (overlap_len, total_len, coverage_percent, src_node_mask_for_viz)
    """
    source_pos = np.asarray(source_pos, float)
    target_pos = np.asarray(target_pos, float)
    if use_xy_only:
        source_pos = source_pos[:, :2]
        target_pos = target_pos[:, :2]

    # Resample both polylines
    src_rs = resample_polyline(source_pos, step)
    tgt_rs = resample_polyline(target_pos, max(step * 0.5, 1e-3))  # slightly denser target

    # Edge cases
    if src_rs.shape[0] < 2:
        return 0.0, 0.0, 0.0, np.zeros(source_pos.shape[0], dtype=bool)

    # KD-tree on target samples
    try:
        from scipy.spatial import cKDTree
        tree = cKDTree(tgt_rs)
        def dmin(P):  # min distance from points P to target polyline samples
            return tree.query(P, k=1, workers=-1)[0]
    except Exception:
        # NumPy fallback
        def dmin(P):
            # pairwise distances to target samples
            diff = P[:, None, :] - tgt_rs[None, :, :]
            return np.linalg.norm(diff, axis=2).min(axis=1)

    # Segment-wise test: count length if either endpoint or midpoint is within radius
    seg_vec = np.diff(src_rs, axis=0)
    seg_len = np.linalg.norm(seg_vec, axis=1)
    P0 = src_rs[:-1]
    P1 = src_rs[1:]
    Pm = 0.5 * (P0 + P1)

    d0 = dmin(P0)
    d1 = dmin(P1)
    dm = dmin(Pm)
    keep = (d0 <= radius) | (d1 <= radius) | (dm <= radius)
    overlap_len = float(seg_len[keep].sum())
    total_len = float(seg_len.sum())
    coverage = 0.0 if total_len == 0 else 100.0 * overlap_len / total_len

    # For visualization on ORIGINAL source nodes (not resampled):
    # mark nodes whose nearest distance to target is within radius
    try:
        from scipy.spatial import cKDTree
        tree_vis = cKDTree(tgt_rs)
        node_mask = tree_vis.query(source_pos, k=1, workers=-1)[0] <= radius
    except Exception:
        diff = source_pos[:, None, :] - tgt_rs[None, :, :]
        node_mask = np.linalg.norm(diff, axis=2).min(axis=1) <= radius

    return overlap_len, total_len, coverage, node_mask


# def polyline_length(pts: np.ndarray) -> float:
#     """Total length of a path defined by ordered points (N,D)."""
#     if pts is None or len(pts) < 2:
#         return 0.0
#     segs = np.diff(pts, axis=0)                 # (N-1, D)
#     return float(np.linalg.norm(segs, axis=1).sum())

# def overlap_lengths_from_mask(source_pos: np.ndarray, mask: np.ndarray):
#     """
#     Compute lengths along the source trajectory using the overlap mask.
#     Overlap length counts segments where BOTH endpoints are overlapped.
#     Non-overlap length counts segments where BOTH endpoints are non-overlapped.
#     Segments crossing the boundary are ignored (discrete approximation).
#     """
#     if source_pos.shape[0] < 2:
#         return 0.0, 0.0, 0.0
#     segs = np.diff(source_pos, axis=0)          # (N-1, D)
#     seg_len = np.linalg.norm(segs, axis=1)      # (N-1,)
#     edge_mask_overlap = mask[:-1] & mask[1:]
#     edge_mask_non     = (~mask[:-1]) & (~mask[1:])
#     overlap_len = float(seg_len[edge_mask_overlap].sum())
#     non_overlap_len = float(seg_len[edge_mask_non].sum())
#     total_len = float(seg_len.sum())
#     return overlap_len, non_overlap_len, total_len

# def check_within_bounds(max_overlap_dist: float,
#                         p1: np.ndarray,
#                         p2: np.ndarray) -> bool:
#     """Euclidean distance threshold."""
#     return np.linalg.norm(np.asarray(p1) - np.asarray(p2)) <= max_overlap_dist


# def get_overlap_points(max_overlap_dist: float,
#                        source_pos: np.ndarray,
#                        target_pos: np.ndarray,
#                        mode: str = "nearest"):
#     """
#     Find overlaps between source_pos and target_pos independent of time.

#     Parameters
#     ----------
#     max_overlap_dist : float
#         Distance threshold.
#     source_pos : (Ns, D) array
#         Source positions.
#     target_pos : (Nt, D) array
#         Target positions.
#     mode : {"nearest", "all_pairs"}
#         - "nearest": for each source point, take its nearest target (if within threshold).
#         - "all_pairs": return *every* (src_i, tgt_j) pair within threshold.

#     Returns
#     -------
#     dict
#         If mode == "nearest":
#             {
#               "src_idx": (K,), "tgt_idx": (K,),
#               "src": (K,D), "tgt": (K,D),
#               "dist": (K,), "mask": (Ns,)  # True where a match was found
#             }
#         If mode == "all_pairs":
#             {
#               "src_idx": (K,), "tgt_idx": (K,),
#               "src": (K,D), "tgt": (K,D),
#               "dist": (K,)
#             }
#     """
#     source_pos = np.asarray(source_pos, dtype=float)
#     target_pos = np.asarray(target_pos, dtype=float)
#     if source_pos.ndim != 2 or target_pos.ndim != 2:
#         raise ValueError("source_pos and target_pos must be (N, D) arrays.")
#     if source_pos.shape[1] != target_pos.shape[1]:
#         raise ValueError("Dimensionality mismatch between source_pos and target_pos.")

#     Ns, D = source_pos.shape
#     Nt = target_pos.shape[0]

#     # Try to use a KD-tree if SciPy is available; fall back to NumPy otherwise.
#     try:
#         from scipy.spatial import cKDTree  # type: ignore
#         tree = cKDTree(target_pos)

#         if mode == "nearest":
#             dists, idxs = tree.query(source_pos, k=1, workers=-1)
#             mask = dists <= max_overlap_dist
#             mask_bad = dists > max_overlap_dist
#             K = int(np.count_nonzero(mask))
#             if K == 0:
#                 return {
#                     "src_idx": np.array([], dtype=int),
#                     "tgt_idx": np.array([], dtype=int),
#                     "src": np.empty((0, D)),
#                     "tgt": np.empty((0, D)),
#                     "dist": np.array([], dtype=float),
#                     "mask": np.zeros(Ns, dtype=bool),
#                 }
#             return {
#                 "src_idx": np.nonzero(mask)[0],
#                 "tgt_idx": idxs[mask].astype(int),
#                 "src": source_pos[mask],
#                 "src_not": source_pos[mask_bad],
#                 "tgt": target_pos[idxs[mask]],
#                 "dist": dists[mask],
#                 "mask": mask,
#             }

#         elif mode == "all_pairs":
#             pairs = tree.query_ball_point(source_pos, r=max_overlap_dist, workers=-1)
#             src_idx, tgt_idx = [], []
#             for i, js in enumerate(pairs):
#                 for j in js:
#                     src_idx.append(i)
#                     tgt_idx.append(j)
#             if not src_idx:
#                 return {
#                     "src_idx": np.array([], dtype=int),
#                     "tgt_idx": np.array([], dtype=int),
#                     "src": np.empty((0, D)),
#                     "tgt": np.empty((0, D)),
#                     "dist": np.array([], dtype=float),
#                 }
#             src_idx = np.asarray(src_idx, dtype=int)
#             tgt_idx = np.asarray(tgt_idx, dtype=int)
#             diffs = source_pos[src_idx] - target_pos[tgt_idx]
#             dists = np.linalg.norm(diffs, axis=1)
#             return {
#                 "src_idx": src_idx,
#                 "tgt_idx": tgt_idx,
#                 "src": source_pos[src_idx],
#                 "tgt": target_pos[tgt_idx],
#                 "dist": dists,
#             }

#         else:
#             raise ValueError("mode must be 'nearest' or 'all_pairs'")

#     except Exception:  # No SciPy or tree build failed → NumPy fallback
#         if mode == "nearest":
#             # Compute pairwise distances efficiently: ||a-b||^2 = ||a||^2 + ||b||^2 - 2 a·b
#             A = source_pos
#             B = target_pos
#             A2 = np.sum(A*A, axis=1, keepdims=True)           # (Ns,1)
#             B2 = np.sum(B*B, axis=1)[None, :]                 # (1,Nt)
#             D2 = A2 + B2 - 2.0 * (A @ B.T)                    # (Ns,Nt)
#             D2 = np.maximum(D2, 0.0)
#             idxs = np.argmin(D2, axis=1)
#             dists = np.sqrt(D2[np.arange(Ns), idxs])
#             mask = dists <= max_overlap_dist
#             if not np.any(mask):
#                 return {
#                     "src_idx": np.array([], dtype=int),
#                     "tgt_idx": np.array([], dtype=int),
#                     "src": np.empty((0, D)),
#                     "tgt": np.empty((0, D)),
#                     "dist": np.array([], dtype=float),
#                     "mask": np.zeros(Ns, dtype=bool),
#                 }
#             return {
#                 "src_idx": np.nonzero(mask)[0],
#                 "tgt_idx": idxs[mask].astype(int),
#                 "src": source_pos[mask],
#                 "tgt": target_pos[idxs[mask]],
#                 "dist": dists[mask],
#                 "mask": mask,
#             }

#         elif mode == "all_pairs":
#             # Brute-force all pair distances (watch memory for large N)
#             A = source_pos[:, None, :]        # (Ns,1,D)
#             B = target_pos[None, :, :]        # (1,Nt,D)
#             diffs = A - B                     # (Ns,Nt,D)
#             dists = np.linalg.norm(diffs, axis=2)  # (Ns,Nt)
#             mask = dists <= max_overlap_dist
#             src_idx, tgt_idx = np.nonzero(mask)
#             if src_idx.size == 0:
#                 return {
#                     "src_idx": np.array([], dtype=int),
#                     "tgt_idx": np.array([], dtype=int),
#                     "src": np.empty((0, D)),
#                     "tgt": np.empty((0, D)),
#                     "dist": np.array([], dtype=float),
#                 }
#             return {
#                 "src_idx": src_idx,
#                 "tgt_idx": tgt_idx,
#                 "src": source_pos[src_idx],
#                 "tgt": target_pos[tgt_idx],
#                 "dist": dists[src_idx, tgt_idx],
#             }

#         else:
#             raise ValueError("mode must be 'nearest' or 'all_pairs'")


# def interpolate_pose(query_t: float,
#                      ts: np.ndarray,
#                      poses: np.ndarray) -> np.ndarray:
#     """
#     Linearly interpolate a pose (position vector) at query_t.

#     Parameters
#     ----------
#     query_t : float
#         Timestamp to interpolate at.
#     ts : (N,) array
#         Monotonic timestamps corresponding to `poses`.
#     poses : (N, D) array
#         Pose positions (e.g., x,y or x,y,z) at each timestamp.

#     Returns
#     -------
#     (D,) array
#         Interpolated pose at query_t. Clamps to endpoints if out of range.
#     """
#     ts = np.asarray(ts)
#     poses = np.asarray(poses)

#     if ts.ndim != 1:
#         raise ValueError("ts must be 1D.")
#     if poses.ndim != 2 or poses.shape[0] != ts.shape[0]:
#         raise ValueError("poses must be shape (N, D) with poses.shape[0] == ts.shape[0].")

#     # Clamp to endpoints
#     if query_t <= ts[0]:
#         return poses[0]
#     if query_t >= ts[-1]:
#         return poses[-1]

#     # Find interval [i-1, i] such that ts[i-1] <= query_t < ts[i]
#     i = np.searchsorted(ts, query_t, side='right')
#     t0, t1 = ts[i-1], ts[i]
#     p0, p1 = poses[i-1], poses[i]

#     dt = (t1 - t0)
#     if dt == 0:
#         # Duplicate timestamps; fall back to left pose
#         return p0.copy()

#     alpha = (query_t - t0) / dt
#     return (1.0 - alpha) * p0 + alpha * p1


# def check_within_bounds(max_overlap_dist: float,
#                         pose1: np.ndarray,
#                         pose2: np.ndarray) -> bool:
#     """
#     True if Euclidean distance between pose1 and pose2 is <= max_overlap_dist.
#     """
#     pose1 = np.asarray(pose1)
#     pose2 = np.asarray(pose2)
#     return np.linalg.norm(pose1 - pose2) <= max_overlap_dist


# def get_overlap_points(max_overlap_dist: float,
#                        source_ts: np.ndarray,
#                        source_pos: np.ndarray,
#                        target_ts: np.ndarray,
#                        target_pos: np.ndarray) -> Dict[str, np.ndarray]:
#     """
#     For each (time, position) in the source, interpolate target at the same time,
#     and keep those whose distance is within max_overlap_dist.

#     Parameters
#     ----------
#     max_overlap_dist : float
#         Distance threshold.
#     source_ts : (Ns,) array
#         Source timestamps.
#     source_pos : (Ns, D) array
#         Source positions.
#     target_ts : (Nt,) array
#         Target timestamps.
#     target_pos : (Nt, D) array
#         Target positions.

#     Returns
#     -------
#     dict with keys:
#         'time'   : (K,) array of kept times
#         'src'    : (K, D) array of source positions
#         'tgt'    : (K, D) array of interpolated target positions
#         'dist'   : (K,) array of distances
#         'mask'   : (Ns,) boolean mask of which source rows were kept
#     """
#     source_ts = np.asarray(source_ts)
#     source_pos = np.asarray(source_pos)
#     target_ts = np.asarray(target_ts)
#     target_pos = np.asarray(target_pos)

#     if source_ts.ndim != 1 or target_ts.ndim != 1:
#         raise ValueError("source_ts and target_ts must be 1D.")
#     if source_pos.shape[0] != source_ts.shape[0]:
#         raise ValueError("source_pos must have same length as source_ts.")
#     if target_pos.shape[0] != target_ts.shape[0]:
#         raise ValueError("target_pos must have same length as target_ts.")
#     if source_pos.ndim != 2 or target_pos.ndim != 2 or source_pos.shape[1] != target_pos.shape[1]:
#         raise ValueError("source_pos and target_pos must be (N, D) with the same D.")

#     kept_times: List[float] = []
#     kept_src: List[np.ndarray] = []
#     kept_tgt: List[np.ndarray] = []
#     kept_dist: List[float] = []
#     mask = np.zeros(len(source_ts), dtype=bool)

#     for idx, (s_time, s_pos) in enumerate(zip(source_ts, source_pos)):
#         t_pos_interp = interpolate_pose(s_time, target_ts, target_pos)
#         dist = np.linalg.norm(s_pos - t_pos_interp)
#         if dist <= max_overlap_dist:
#             kept_times.append(s_time)
#             kept_src.append(s_pos)
#             kept_tgt.append(t_pos_interp)
#             kept_dist.append(dist)
#             mask[idx] = True

#     if kept_times:
#         return {
#             "time": np.array(kept_times),
#             "src": np.vstack(kept_src),
#             "tgt": np.vstack(kept_tgt),
#             "dist": np.array(kept_dist),
#             "mask": mask,
#         }
#     else:
#         # Return empty arrays with correct shapes
#         D = source_pos.shape[1]
#         return {
#             "time": np.array([], dtype=float),
#             "src": np.empty((0, D), dtype=float),
#             "tgt": np.empty((0, D), dtype=float),
#             "dist": np.array([], dtype=float),
#             "mask": mask,
#         }
    
# # def interpolate_pose(ts, poses, timestamps):

# # def check_within_bounds(max_overlap_dist, pose1, pose2):
# #     return np.sqrt(pose1**pose1 - pose2**pose2) < max_overlap_dist

# # def get_overlap_points(max_overlap_dist, source_ts, source_pos, target_ts, target_pos):
# #     """ """
# #     for s_time, s_pos in zip(source_ts, source_pos):
# #         t_pos_interp = interpolate_pose(s_time, target_ts, target_pos)
# #         is_within = check_within_bounds(max_overlap_dist, s_pos, t_pos_interp)

def load_poses(csv_path):
    """Reads timestamp, x,y,z,qx,qy,qz,qw from CSV, skipping comments."""
    timestamps = []
    xs, ys, zs = [], [], []
    qxs, qys, qzs, qws = [], [], [], []
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith('#'):
                continue
            t, x, y, z, qx, qy, qz, qw = map(float, row)
            timestamps.append(t)
            xs.append(x); ys.append(y); zs.append(z)
            qxs.append(qx); qys.append(qy); qzs.append(qz); qws.append(qw)
    positions = np.vstack((xs, ys, zs)).T
    quats  = np.vstack((qws, qxs, qys, qzs)).T  # Open3D expects [w, x, y, z]
    return timestamps, positions, quats


def get_pcd_positions(positions, color):
    # Create point cloud of positions
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(positions)

    # Optionally color the trajectory (here: red line)
    colors = np.tile(color, (len(positions), 1))
    pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd


def main():
    DATA_ROOT = os.getenv("CU_MULTI_ROOT")
    if DATA_ROOT is None:
        sys.exit(
            "ERROR: Environment variable CU_MULTI_ROOT is not set.\n"
            "Please set it before running the script, e.g.:\n"
            "  export CU_MULTI_ROOT=/your/custom/path\n"
        )
    print("CU_MULTI_ROOT is:", DATA_ROOT)

    environments = ["kittredge_loop", "main_campus"]
    robots = [1, 2, 3, 4]
    colors = [
        np.array([87, 227, 137])/255.0,
        np.array([192, 97, 203])/255.0,
        np.array([255, 163, 72])/255.0,
        np.array([98, 160, 234])/255.0,
    ]

    target_robot = 3
    max_overlap_dist = 3 # In meters. Max dist two robots can be to say they have overlap

    # vis = o3d.visualization.Visualizer()
    # vis.create_window(window_name="Robot Trajectory + Poses")

    for environment in environments:
        for source_robot, color in zip(robots, colors):
            vis = o3d.visualization.Visualizer()
            vis.create_window(window_name="Robot Trajectory + Poses")
    
            source_file_path = os.path.join(DATA_ROOT, "gt_poses", f"{environment}_robot{source_robot}_ref.csv")
            source_ts, source_pos, source_quats = load_poses(source_file_path)

            target_file_path = os.path.join(DATA_ROOT, "gt_poses", f"{environment}_robot{target_robot}_ref.csv")
            target_ts, target_pos, target_quats = load_poses(target_file_path)

            # overlap_points = get_overlap_points(max_overlap_dist, source_pos, target_pos)

            # # Compute lengths
            # mask = overlap_points["mask"]
            # ov_len, non_len, tot_len = overlap_lengths_from_mask(source_pos, mask)
            # coverage = 0.0 if tot_len == 0 else (ov_len / tot_len) * 100.0
            # print(f"[{environment}] src robot {source_robot} vs target {target_robot}: "
            # f"overlap = {ov_len:.2f} m, non-overlap = {non_len:.2f} m, "
            # f"total = {tot_len:.2f} m, coverage = {coverage:.1f}%")

            # src_overlap = overlap_points.get("src")
            # src_overlap[:, 2] += 2
            # source_path_pcd = get_pcd_positions(src_overlap, color)
            # vis.add_geometry(source_path_pcd)

            # src_not_overlap = overlap_points.get("src_not")
            # src_not_overlap[:, 2] += 2
            # source_not_path_pcd = get_pcd_positions(src_not_overlap, [0,0,0])
            # vis.add_geometry(source_not_path_pcd)

            # t_color = colors[target_robot-1]
            # target_path_pcd = get_pcd_positions(target_pos, t_color)
            # vis.add_geometry(target_path_pcd)

            # vis.run()
            # vis.destroy_window()
            overlap_len, total_len, coverage, node_mask = overlap_length_spatial(
                source_pos, target_pos,
                radius=max_overlap_dist,
                step=0.25,          # tune for accuracy/speed; smaller = more accurate
                use_xy_only=True,   # or False if you want full XYZ distance
            )

            # print(f"[{environment}] R{source_robot} vs R{target_robot}: "
            #     f"overlap={overlap_len:.2f} m / total={total_len:.2f} m ({coverage:.1f}%)")

            stats = symmetric_overlap_stats(source_pos, target_pos, radius=max_overlap_dist, step=0.25, use_xy_only=True)

            print(f"[{environment}] R{source_robot} ↔ R{target_robot}: "
                f"sym_len={stats['sym_len_m']:.2f} m, "
                f"Dice={stats['dice_pct']:.1f}%, "
                f"AvgCov={stats['avg_cov_pct']:.1f}% "
                f"(A→B {stats['A_cov_pct']:.1f}%, B→A {stats['B_cov_pct']:.1f}%)")

            # Visualization: split original source nodes by mask (lift Z by +2)
            src_overlap = source_pos[node_mask].copy()
            if src_overlap.size:
                src_overlap = np.pad(src_overlap, ((0,0),(0, max(0, 3 - src_overlap.shape[1]))))  # ensure 3D
                src_overlap[:, 2] += 2
                vis.add_geometry(get_pcd_positions(src_overlap, color))

            src_not_overlap = source_pos[~node_mask].copy()
            if src_not_overlap.size:
                src_not_overlap = np.pad(src_not_overlap, ((0,0),(0, max(0, 3 - src_not_overlap.shape[1]))))
                src_not_overlap[:, 2] += 2
                vis.add_geometry(get_pcd_positions(src_not_overlap, [0,0,0]))

            # Target path (unchanged)
            t_color = colors[target_robot-1]
            vis.add_geometry(get_pcd_positions(target_pos, t_color))


if __name__ == "__main__":
    main()
