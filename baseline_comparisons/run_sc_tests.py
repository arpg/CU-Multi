#!/usr/bin/env python3
import os
import sys
import csv
import re
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import open3d as o3d
import pyvista as pv
from tqdm import tqdm
import json


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





# --------------------------- Utilities ---------------------------

def _natural_sort_key(s):
    """Sort '000123.bin', '2.bin', '10.bin' in human order."""
    return [int(t) if t.isdigit() else t.lower() for t in re.split(r'(\d+)', s)]


def distance_sc(sc1, sc2):
    """
    Fast ScanContext distance using FFT-based circular correlation across sectors.
    Normalizes columns -> cosine per column, then finds the best circular shift.
    Returns a distance in [0, 1].
    """
    sc1 = np.asarray(sc1, dtype=np.float32)
    sc2 = np.asarray(sc2, dtype=np.float32)
    if sc1.shape != sc2.shape:
        raise ValueError(f"SC shapes must match, got {sc1.shape} vs {sc2.shape}")
    R, S = sc1.shape

    # Column norms (across rings)
    n1 = np.linalg.norm(sc1, axis=0)
    n2 = np.linalg.norm(sc2, axis=0)
    mask1 = (n1 > 0).astype(np.float32)
    mask2 = (n2 > 0).astype(np.float32)

    # Normalize columns to unit-length; empty columns become zeros
    A = sc1 / np.where(n1 > 0, n1, 1.0)
    B = sc2 / np.where(n2 > 0, n2, 1.0)

    # FFT along sector axis for each ring
    FA = np.fft.fft(A, axis=1)
    FB = np.fft.fft(B, axis=1)

    # Circular cross-correlation per ring: ifft(conj(FA) * FB)
    # Sum across rings -> per-shift total cosine sum
    corr_per_ring = np.fft.ifft(np.conj(FA) * FB, axis=1).real  # (R, S)
    cos_sum_per_shift = corr_per_ring.sum(axis=0)               # (S,)

    # Compute, per shift, how many column pairs are valid (non-empty in both)
    mA = mask1[None, :]  # shape (1, S)
    mB = mask2[None, :]
    FM_A = np.fft.fft(mA, axis=1)
    FM_B = np.fft.fft(mB, axis=1)
    valid_counts = np.fft.ifft(np.conj(FM_A) * FM_B, axis=1).real[0]  # (S,)

    # Avoid div-by-zero; counts should be ~integers after IFFT
    valid_counts = np.maximum(np.rint(valid_counts), 1.0)

    sims = cos_sum_per_shift / valid_counts  # mean cosine per shift
    sim = float(np.max(sims))
    dist = 1.0 - sim
    return float(np.clip(dist, 0.0, 1.0))


# def distance_sc(sc1, sc2):
#     """
#     ScanContext distance with circular column shift invariance.
#     Computes cosine similarity per column and averages over columns that are non-empty in BOTH SCs.
#     Returns a distance in [0, 1].
#     """
#     if sc1.shape != sc2.shape:
#         raise ValueError(f"SC shapes must match, got {sc1.shape} vs {sc2.shape}")

#     num_sectors = sc1.shape[1]
#     sims = np.zeros(num_sectors, dtype=float)

#     # precompute column norms for sc2 once
#     sc2_norms = np.linalg.norm(sc2, axis=0)

#     for shift in range(num_sectors):
#         rolled = np.roll(sc1, shift, axis=1)
#         rolled_norms = np.linalg.norm(rolled, axis=0)

#         # valid columns must have nonzero norm in BOTH
#         valid = (rolled_norms > 0) & (sc2_norms > 0)
#         if not np.any(valid):
#             sims[shift] = 0.0
#             continue

#         # cosine similarity per valid column
#         dot = np.sum(rolled[:, valid] * sc2[:, valid], axis=0)
#         cos = dot / (rolled_norms[valid] * sc2_norms[valid])
#         # numerical clamp
#         cos = np.clip(cos, -1.0, 1.0)

#         sims[shift] = np.mean(cos)

#     sim = float(np.max(sims))
#     dist = 1.0 - sim
#     # clamp to [0,1] for safety
#     return float(np.clip(dist, 0.0, 1.0))


class kitti_vlp_database:
    robot_colors = {
        "robot1": np.array([87, 227, 137]) / 255.0,
        "robot2": np.array([192, 97, 203]) / 255.0,
        "robot3": np.array([255, 163, 72]) / 255.0,
        "robot4": np.array([98, 160, 234]) / 255.0,
    }

    def __init__(self, robot_name, bin_dir, poses_file_path):

        self.robot_color = kitti_vlp_database.robot_colors[robot_name]

        self.bin_dir = bin_dir

        # keep only .bin files, natural sort
        self.bin_files = [f for f in os.listdir(bin_dir) if f.endswith(".bin")]
        self.bin_files.sort(key=_natural_sort_key)

        self.num_bins = len(self.bin_files)
        if self.num_bins == 0:
            raise FileNotFoundError(f"No .bin files found in {bin_dir}")

        self.timestamp, self.position, self.quat = load_poses(poses_file_path)


class ScanContext:
    # static variables
    viz = False
    downcell_size = 0.2
    kitti_lidar_height = 0.7

    # choose one resolution for now (extendable)
    sector_res = np.array([60])
    ring_res = np.array([20])
    max_length = 80

    robot_colors = {
        "robot1": np.array([87, 227, 137]) / 255.0,
        "robot2": np.array([192, 97, 203]) / 255.0,
        "robot3": np.array([255, 163, 72]) / 255.0,
        "robot4": np.array([98, 160, 234]) / 255.0,
    }

    def __init__(self, robot_name, bin_dir, bin_file_name):
        self.robot_color = ScanContext.robot_colors.get(robot_name, np.array([0.6, 0.6, 0.6]))
        self.bin_dir = bin_dir
        self.bin_file_name = bin_file_name
        self.bin_path = os.path.join(bin_dir, bin_file_name)

        self.SCs = self.genSCs()  # ensure we keep them on the instance

    def load_velo_scan(self):
        scan = np.fromfile(self.bin_path, dtype=np.float32)
        scan = scan.reshape((-1, 4))
        ptcloud_xyz = scan[:, :3]
        return ptcloud_xyz

    @staticmethod
    def xy2theta(x, y):
        # 0-360 degrees
        return (np.degrees(np.arctan2(y, x)) + 360.0) % 360.0

    def pt2rs(self, point, gap_ring, gap_sector, num_ring, num_sector):
        x, y, z = point
        theta = self.xy2theta(x, y)
        faraway = np.hypot(x, y)

        idx_ring = int(faraway // gap_ring)
        idx_sector = int(theta // gap_sector)

        if idx_ring >= num_ring:
            idx_ring = num_ring - 1
        if idx_sector >= num_sector:
            idx_sector = num_sector - 1
        return idx_ring, idx_sector

    def ptcloud2sc(self, ptcloud, num_sector, num_ring, max_length):
        num_points = ptcloud.shape[0]
        gap_ring = max_length / num_ring
        gap_sector = 360.0 / num_sector

        # allow up to N points per cell, then take max height
        enough_large = 1000
        sc_storage = np.zeros((enough_large, num_ring, num_sector), dtype=np.float32)
        sc_counter = np.zeros((num_ring, num_sector), dtype=np.int32)

        for pt_idx in range(num_points):
            point = ptcloud[pt_idx, :]
            point_height = point[2] + ScanContext.kitti_lidar_height

            idx_ring, idx_sector = self.pt2rs(point, gap_ring, gap_sector, num_ring, num_sector)
            cnt = sc_counter[idx_ring, idx_sector]
            if cnt >= enough_large:
                continue

            sc_storage[cnt, idx_ring, idx_sector] = point_height
            sc_counter[idx_ring, idx_sector] = cnt + 1

        sc = np.amax(sc_storage, axis=0)
        return sc

    def genSCs(self):
        ptcloud_xyz = self.load_velo_scan()
        # print("The number of original points:", ptcloud_xyz.shape)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(ptcloud_xyz)

        downpcd = pcd.voxel_down_sample(voxel_size=ScanContext.downcell_size)
        colors = np.tile(self.robot_color, (len(downpcd.points), 1))
        downpcd.colors = o3d.utility.Vector3dVector(colors)

        ptcloud_xyz_downed = np.asarray(downpcd.points)
        # print("The number of downsampled points:", ptcloud_xyz_downed.shape)

        if ScanContext.viz:
            o3d.visualization.draw_geometries([downpcd])

        SCs = []
        for res in range(len(ScanContext.sector_res)):
            num_sector = int(ScanContext.sector_res[res])
            num_ring = int(ScanContext.ring_res[res])
            sc = self.ptcloud2sc(ptcloud_xyz_downed, num_sector, num_ring, ScanContext.max_length)
            SCs.append(sc)
        return SCs

    def plot_multiple_sc(self, fig_idx=1):
        num_res = len(ScanContext.sector_res)
        fig, axes = plt.subplots(nrows=num_res, squeeze=False)
        axes = axes.ravel()

        axes[0].set_title('Scan Contexts with multiple resolutions', fontsize=14)
        for ax, res in zip(axes, range(num_res)):
            ax.imshow(self.SCs[res])
            ax.set_xlabel(f"{ScanContext.sector_res[res]} sectors, {ScanContext.ring_res[res]} rings")

        plt.tight_layout()
        plt.show()


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
    quats = np.vstack((qws, qxs, qys, qzs)).T  # Open3D expects [w, x, y, z]
    return np.array(timestamps), positions, quats


def save_camera(pl, path):
    """Save current PyVista camera to JSON."""
    cam = pl.camera
    payload = {
        "position": list(map(float, cam.position)),
        "focal_point": list(map(float, cam.focal_point)),
        "view_up": list(map(float, cam.up)),
        "view_angle": float(cam.GetViewAngle()),                 # perspective FOV
        "parallel_projection": bool(cam.GetParallelProjection()),
        "parallel_scale": float(cam.GetParallelScale()),
        "clipping_range": list(map(float, cam.GetClippingRange())),
    }
    with open(path, "w") as f:
        json.dump(payload, f, indent=2)


def load_camera(pl, path):
    """Load camera from JSON and apply to the plotter."""
    with open(path, "r") as f:
        cfg = json.load(f)
    cam = pl.camera
    cam.position = cfg["position"]
    cam.focal_point = cfg["focal_point"]
    cam.up = cfg["view_up"]
    cam.SetParallelProjection(cfg.get("parallel_projection", False))
    cam.SetParallelScale(cfg.get("parallel_scale", cam.GetParallelScale()))
    cam.SetViewAngle(cfg.get("view_angle", cam.GetViewAngle()))
    if "clipping_range" in cfg:
        cam.SetClippingRange(*cfg["clipping_range"])
    # Keep PyVista’s convenience tuple in sync (optional)
    pl.camera_position = (tuple(cam.position), tuple(cam.focal_point), tuple(cam.up))


def plot_pose_matches_pyvista(
    A_positions, A_NOT_positions, B_positions, B_NOT_positions, matches, max_dist,
    title="Pose matches", 
    screenshot_path=None, 
    show=True,
    colorA=[0,0,0], 
    colorB=[0,0,0],
    lift_A=100.0,
    camera_preset_path=None, 
    add_keybinds=True):

    # make 2D footprints and lift B
    A_traj = A_positions.copy()
    A_traj[:, 2] = 0.0
    A_traj = A_traj + np.array([0.0, 0.0, lift_A])

    B_traj = B_positions.copy()
    B_traj[:, 2] = 0.0

    if (A_NOT_positions is not None) and (B_NOT_positions is not None):
        A_NOT_traj = A_NOT_positions.copy()
        B_NOT_traj = B_NOT_positions.copy()

        A_NOT_traj[:, 2] = 0.0
        B_NOT_traj[:, 2] = 0.0

        A_NOT_traj = A_NOT_traj + np.array([0.0, 0.0, lift_A])

    nA = len(A_traj)
    nB = len(B_traj)

    # Build pair list and split into TP/FP by XY distance (no lift)
    pairs_tp, pairs_fp = [], []
    for i, j in enumerate(matches):
        dist_xy = np.linalg.norm(A_positions[i, :2] - B_positions[j, :2])
        if dist_xy <= max_dist:
            pairs_tp.append([i, nA + j])
        else:
            pairs_fp.append([i, nA + j])

    # Combine points so we can reference by index
    P = np.vstack([A_traj, B_traj])

    def poly_from_pairs(points, pairs):
        if not pairs:
            return None
        # Each segment uses 2 points: build a PolyData with "lines" connectivity
        pts = np.vstack([[points[i], points[j]] for i, j in pairs])
        # lines array: [2, 0,1,  2, 2,3,  2, 4,5, ...]
        lines = np.empty(3 * len(pairs), dtype=np.int64)
        lines[0::3] = 2
        lines[1::3] = np.arange(0, 2 * len(pairs), 2, dtype=np.int64)
        lines[2::3] = np.arange(1, 2 * len(pairs), 2, dtype=np.int64)
        mesh = pv.PolyData(pts)
        mesh.lines = lines
        return mesh

    segs_tp = poly_from_pairs(P, pairs_tp)
    segs_fp = poly_from_pairs(P, pairs_fp)

    def polyline_from_track(track):
        # Create a single polyline through all points in order
        m = pv.PolyData(track)
        n = len(track)
        if n >= 2:
            # connectivity: [n, 0,1,2,...,n-1]
            conn = np.concatenate(([n], np.arange(n, dtype=np.int64)))
            m.lines = conn
        return m

    # Plotter
    pl = pv.Plotter(off_screen=not show, window_size=(3200, 2400))
    pl.set_background("white")

    # Trajectory points (spheres) and polylines
    A_pts = pv.PolyData(A_traj)
    B_pts = pv.PolyData(B_traj)
    pl.add_mesh(A_pts, render_points_as_spheres=True, point_size=35, color=colorA, name="A pts")
    pl.add_mesh(B_pts, render_points_as_spheres=True, point_size=35, color=colorB, name="B pts")

    if (A_NOT_positions is not None) and (B_NOT_positions is not None):
        A_NOT_pts = pv.PolyData(A_NOT_traj)
        B_NOT_pts = pv.PolyData(B_NOT_traj)
        pl.add_mesh(A_NOT_pts, render_points_as_spheres=True, point_size=35, 
                    color="#00000049", name="A NOT pts")
        pl.add_mesh(B_NOT_pts, render_points_as_spheres=True, point_size=35, 
                    color="#00000049", name="B NOT pts")

    # A_line = polyline_from_track(A_traj)
    # B_line = polyline_from_track(B_traj)
    # pl.add_mesh(A_line, line_width=2, color="#1f77b4", name="A path")
    # pl.add_mesh(B_line, line_width=2, color="#d62728", name="B path")

    # Matching segments: green = within max_dist, red = outside
    if segs_tp is not None:
        pl.add_mesh(segs_tp, color="#00ff00", line_width=3, opacity=0.6, name="Matches (TP)")
    if segs_fp is not None:
        pl.add_mesh(segs_fp, color="#ff0000", line_width=3, opacity=0.9, name="Matches (FP)")

    # Axes/grid/view
    # pl.add_axes(line_width=5)
    # pl.show_bounds(grid="back", location="outer")
    # pl.add_text(title, font_size=14, color="black")
    # pl.view_xy()  # top-down is usually nicest for XY matching
    # Load camera preset (if provided)

    if camera_preset_path and os.path.exists(camera_preset_path):
        load_camera(pl, camera_preset_path)

    # Optional: handy keybinds to save/load during interaction
    if add_keybinds:
        if camera_preset_path:
            pl.add_text("[S] save cam  [L] load cam", 
                        position="upper_left", font_size=10, color="black")
            pl.add_key_event("s", lambda: (save_camera(pl, camera_preset_path),
                                           print(f"Saved camera → {camera_preset_path}")))
            pl.add_key_event("l", lambda: (load_camera(pl, camera_preset_path), 
                                           pl.render(),
                                           print(f"Loaded camera ← {camera_preset_path}")))
        # Quick zoom keys (optional)
        pl.add_key_event("+", lambda: (pl.camera.Zoom(1.1), pl.render()))
        pl.add_key_event("-", lambda: (pl.camera.Zoom(1/1.1), pl.render()))

    # Render & export
    pl.enable_anti_aliasing()
    pl.show(auto_close=False)
    # pl.show(title=title, auto_close=False)        # interactive or off-screen render

    if screenshot_path:
        pl.screenshot(screenshot_path, window_size=(3200, 2400))

    return pl.show() if show else None


def get_robot_databases(DATA_ROOT, robots, env):
    # load databases
    robot_DB = {}
    for robot in robots:
        robot_dir = os.path.join(DATA_ROOT, f"{env}/robot{robot}")
        lidar_bin_dir = os.path.join(robot_dir, "lidar_bin", "data")
        poses_file_path = os.path.join(DATA_ROOT, f"{env}", f"robot{robot}", 
                                       f"{env}_robot{robot}_ref.csv")
        robot_DB[f"robot{robot}"] = kitti_vlp_database(robot_name=f"robot{robot}", 
                                                       bin_dir=lidar_bin_dir, 
                                                       poses_file_path=poses_file_path)

    return robot_DB


recall_grid = np.linspace(0.0, 1.0, 101)

def pr_envelope(prec: np.ndarray) -> np.ndarray:
    pe = prec.copy()
    for i in range(len(pe) - 2, -1, -1):
        pe[i] = max(pe[i], pe[i + 1])
    return pe


def interp_pr_to_grid(rec: np.ndarray, prec: np.ndarray, grid: np.ndarray) -> np.ndarray:
    if rec.size == 0:
        return np.zeros_like(grid)
    pe = pr_envelope(prec)
    if rec[0] > 0:
        rec_i = np.concatenate(([0.0], rec))
        pe_i  = np.concatenate(([pe[0]], pe))
    else:
        rec_i, pe_i = rec, pe
    return np.interp(grid, rec_i, pe_i, left=pe_i[0], right=pe_i[-1])


# --------------------------- Main ---------------------------
if __name__ == "__main__":
    DATA_ROOT = os.getenv("CU_MULTI_ROOT")
    if DATA_ROOT is None:
        sys.exit("ERROR: CU_MULTI_ROOT not set")

    environments = ["kittredge_loop"] # ["main_campus", "kittredge_loop"]
    robots = [1, 2, 3, 4]
    comparison_types = ["multi_robot"] # ["multi_session", "multi_robot"]
    render_plot = False
    max_dist = 10
    step = 10
    for comparison_type in comparison_types:
        data_dir = os.path.join("results", comparison_type)
        for env in environments:
            robot_DB = get_robot_databases(DATA_ROOT, robots, env)
            env_prec_list = []
            env_rec_list = []
            env_recall_match_list = []
            env_prec_match_list = []

            # For this env + comparison_type: collect each robot's mean PR curve
            robot_mean_curves = {}          # robot_id -> (recall_grid, mean_precision_on_grid)
            robot_mean_op_pts = {}          # robot_id -> (mean_recall_match, mean_precision_match)
            env_dir = os.path.join(data_dir, env)
            os.makedirs(env_dir, exist_ok=True)
            
            for robot_index, robotA in enumerate(robots):
                robot_prec_list = []
                robot_rec_list = []
                robot_recall_match_list = []
                robot_prec_match_list = []
                robot_ave_prec = None
                robot_ave_rec_ = None
                robot_ave_prec_match = None
                robot_ave_rec_match = None

                for robotB in robots:
                    # Dont compare intra descriptors, as they are just too dense
                    if robotA == robotB:
                        continue

                    print(f"Comparing robot{robotA} with robot{robotB}")
                    A_db = robot_DB[f"robot{robotA}"]
                    B_db = robot_DB[f"robot{robotB}"]

                    # max_scans = min(A_db.num_bins, B_db.num_bins)
                    A_indices = np.asarray(range(0, A_db.num_bins, step))
                    B_indices = np.asarray(range(0, B_db.num_bins, step))

                    # positions sampled consistently with indices above
                    A_positions = A_db.position[A_indices]
                    B_positions = B_db.position[B_indices]

                    A_trans_no_overlap = None
                    B_trans_no_overlap = None
                    if comparison_type == "multi_session":
                        _, _, _, mask_A = overlap_length_spatial(
                            A_positions, B_positions,
                            radius=max_dist,
                            step=0.25,          # tune for accuracy/speed; smaller = more accurate
                            use_xy_only=True,   # or False if you want full XYZ distance
                        )
                        _, _, _, mask_B = overlap_length_spatial(
                            B_positions, A_positions,
                            radius=max_dist,
                            step=0.25,          # tune for accuracy/speed; smaller = more accurate
                            use_xy_only=True,   # or False if you want full XYZ distance
                        )

                        A_indices = A_indices[mask_A]  
                        A_trans_no_overlap = A_positions[~mask_A]
                        A_positions = A_positions[mask_A]

                        B_indices = B_indices[mask_B]  
                        B_trans_no_overlap = B_positions[~mask_B]
                        B_positions = B_positions[mask_B]

                    # cache SCs so we don't re-read files repeatedly
                    def compute_scs(db, indices, robot_tag):
                        print(f"\nComputing ScanContext descriptors for {robot_tag}:")
                        cache = {}
                        for idx in tqdm(indices):
                            sc_obj = ScanContext(robot_tag, db.bin_dir, db.bin_files[idx])
                            cache[idx] = sc_obj.SCs[0]  # first resolution
                        return cache

                    A_scs = compute_scs(A_db, A_indices, f"robot{robotA}")
                    B_scs = compute_scs(B_db, B_indices, f"robot{robotB}")

                    print("\nSolving distance matrix:")
                    # distance matrix
                    dist_matrix = np.zeros((len(A_indices), len(B_indices)), dtype=float)
                    for i, ia in tqdm(enumerate(A_indices)):
                        for j, jb in enumerate(B_indices):
                            dist_matrix[i, j] = distance_sc(A_scs[ia], B_scs[jb])

                    print("\nFinding arg min.")
                    # For each A, best B
                    matches = np.argmin(dist_matrix, axis=1)

                    print("\nPLOTTING NOW\n")
                    print("\nPLOTTING NOW (PyVista)\n")
                    screenshot_png = f"LPR_matches_robot{robotA}_to_robot{robotB}.png"
                    screenshot_path = os.path.join(env_dir, screenshot_png)
                    preset = f"camera_robot.json"
                    plot_pose_matches_pyvista(
                        A_positions, 
                        A_trans_no_overlap, 
                        B_positions, 
                        B_trans_no_overlap, 
                        matches, 
                        max_dist,
                        title=f"Pose matches: robot{robotA} vs robot{robotB}",
                        screenshot_path=screenshot_path,
                        show=render_plot,                    # set False for headless/CI
                        colorA=A_db.robot_color, 
                        colorB=B_db.robot_color,
                        lift_A=100.0,
                        camera_preset_path=preset,    # ← persisted camera
                    )
                    print(f"Saved: {screenshot_png}")

                    # ---------- Precision–Recall (no sklearn) ----------
                    # Ground-truth positives: pairs within max_dist in XY
                    dists_xy = np.linalg.norm(
                        A_positions[:, None, :2] - B_positions[None, :, :2], axis=2
                    )
                    y_true = (dists_xy <= max_dist).ravel().astype(np.uint8)   # shape: (len(A)*len(B),)

                    # Similarity score: higher is better
                    y_score = (-dist_matrix).ravel()

                    # Sort by score descending
                    order = np.argsort(-y_score)
                    y_true_sorted = y_true[order]

                    # Cumulate TPs/FPs
                    tp = np.cumsum(y_true_sorted)
                    fp = np.cumsum(1 - y_true_sorted)
                    pos_total = int(y_true.sum())

                    # Avoid div-by-zero
                    prec = tp / np.maximum(tp + fp, 1)
                    rec = tp / max(pos_total, 1)

                    # Build precision envelope (monotone decreasing) and compute AP like sklearn:
                    # AP = sum over recall steps of (R_i - R_{i-1}) * P_interp_i
                    # where P_interp is precision after making it monotonically non-increasing.
                    prec_envelope = prec.copy()
                    for i in range(len(prec_envelope) - 2, -1, -1):
                        prec_envelope[i] = max(prec_envelope[i], prec_envelope[i + 1])

                    # Points where recall changes
                    # (prepend a zero for recall to get deltas cleanly)
                    rec_shifted = np.concatenate(([0.0], rec))
                    # indices where recall increases
                    chg = np.where(np.diff(rec_shifted) > 0)[0]
                    ap = float(np.sum((rec[chg] - rec_shifted[chg]) * prec_envelope[chg]))

                    # Single operating point for your current policy (argmin per A)
                    tp_match = int(np.sum(dists_xy[np.arange(len(A_positions)), matches] <= max_dist))
                    fp_match = int(len(matches) - tp_match)
                    precision_match = tp_match / max(tp_match + fp_match, 1)
                    recall_match = tp_match / max(pos_total, 1)

                    robot_prec_list.append(prec)
                    robot_rec_list.append(rec)
                    robot_prec_match_list.append(precision_match)
                    robot_recall_match_list.append(recall_match)

                # ------ after finishing: for robotB in robots ------
                if robot_prec_list:
                    # Interpolate each pair's PR onto the shared recall grid, then average
                    interp_list = []
                    for prec_i, rec_i in zip(robot_prec_list, robot_rec_list):
                        interp_list.append(interp_pr_to_grid(rec_i, prec_i, recall_grid))
                    interp_mat = np.vstack(interp_list)              # (num_partners, 101)
                    mean_prec_curve = interp_mat.mean(axis=0)        # (101,)

                    # Mean operating point across partners (optional but useful for markers)
                    mean_prec_match = float(np.mean(robot_prec_match_list)) if robot_prec_match_list else 0.0
                    mean_rec_match  = float(np.mean(robot_recall_match_list)) if robot_recall_match_list else 0.0

                    robot_mean_curves[robotA] = (recall_grid, mean_prec_curve)
                    robot_mean_op_pts[robotA] = (mean_rec_match, mean_prec_match)
                else:
                    print(f"[WARN] No PR pairs for env={env}, {comparison_type}, robot{robotA}")

            # ------ after finishing: for robot_index, robotA in enumerate(robots) ------
            if robot_mean_curves:
                fig, ax = plt.subplots(figsize=(7, 5))
                for r_id in sorted(robot_mean_curves.keys()):
                    rec_g, prec_mean = robot_mean_curves[r_id]
                    robot_color = robot_DB[f"robot{r_id}"].robot_color  # use stored robot color
                    ax.plot(rec_g, prec_mean, label=f"robot{r_id}", color=robot_color)

                    # # Optional: draw mean operating point
                    # if r_id in robot_mean_op_pts:
                    #     r_rec, r_prec = robot_mean_op_pts[r_id]
                    #     ax.plot([r_rec], [r_prec], 'o', label=f"", color=robot_color)

                ax.set_xlabel("Recall")
                ax.set_ylabel("Precision")
                ax.set_title(f"{env} | {comparison_type}\nMean PR per robot (averaged across partners)")
                ax.set_xlim(0, 1); ax.set_ylim(0, 1)
                ax.legend(title="Robots")
                ax.grid(False)
                plt.tight_layout()

                out_png = os.path.join(env_dir, f"PR_mean_per_robot_{env}_{comparison_type}.png")
                plt.savefig(out_png, dpi=300, bbox_inches="tight")
                plt.close()
                print(f"[PLOT] Saved: {out_png}")

                # (Optional) also dump curves to CSV for later analysis
                csv_path = os.path.join(env_dir, f"PR_mean_per_robot_{env}_{comparison_type}.csv")
                with open(csv_path, "w") as f:
                    header = ["recall"] + [f"robot{r}" for r in sorted(robot_mean_curves.keys())]
                    f.write(",".join(header) + "\n")
                    for i in range(len(recall_grid)):
                        row = [f"{recall_grid[i]:.6f}"] + [f"{robot_mean_curves[r][1][i]:.6f}" for r in sorted(robot_mean_curves.keys())]
                        f.write(",".join(row) + "\n")
                print(f"[DATA] Saved: {csv_path}")
            else:
                print(f"[INFO] No robot mean curves for {env} | {comparison_type}")

            # # Plot PR curve for each robot in env
            # fig_pr, ax_pr = plt.subplots(figsize=(6, 4))
            # ax_pr.plot(rec, prec, label=f"PR curve (AP={ap:.3f})")
            # ax_pr.plot([recall_match], [precision_match], 'o', label="argmin-per-A point")
            # ax_pr.set_xlabel("Average Recall")
            # ax_pr.set_ylabel("Average Precision")
            # ax_pr.set_title(f"Precision–Recall: robot{robotA} vs robot{robotB}")
            # ax_pr.set_xlim(0, 1)
            # ax_pr.set_ylim(0, 1)
            # ax_pr.grid(False)
            # ax_pr.legend()
            # plt.tight_layout()
            
            # out_name = os.path.join(data_dir, f"Average_PR_robot{robotA}_to_robot{robotB}.png")
            # plt.savefig(out_name, dpi=300, bbox_inches="tight")
            # plt.close()   # close the figure to free memory
            # plt.show()
            # # ---------------------------------------------------