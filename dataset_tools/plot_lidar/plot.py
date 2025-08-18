#!/usr/bin/env python3
import os
import sys
import csv
import numpy as np
import open3d as o3d
import re

def load_lidar(lidar_bin_path):
    data = np.fromfile(lidar_bin_path, dtype=np.float32)
    return data.reshape((-1, 4))


def get_frame_numbers(directory_path) -> list:
    """
    Count the total number of files in the directory
    """
    frame_numbers = []
    all_files = os.listdir(directory_path)

    # Filter out files ending with ".bin" and remove the filetype
    filenames = [
        int(re.search(r'\d+', os.path.splitext(file)[0]).group())
        for file in all_files if file.endswith(".bin") and re.search(r'\d+', os.path.splitext(file)[0])
    ]

    for filename in filenames:
        frame_numbers.append(int(filename))

    return sorted(frame_numbers)


def pose_to_matrix(position, quat_xyzw):
    """
    position: iterable (x, y, z)
    quat_xyzw: iterable (qx, qy, qz, qw)  # change to quat_wxyz if that's your format (see note below)
    returns: 4x4 homogeneous transform matrix
    """
    px, py, pz = position
    qx, qy, qz, qw = quat_xyzw  # <-- If your data is (w, x, y, z), swap to: qw, qx, qy, qz = quat_xyzw

    # Normalize quaternion to be safe
    qn = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if qn == 0:
        raise ValueError("Zero-length quaternion")
    qx, qy, qz, qw = qx/qn, qy/qn, qz/qn, qw/qn

    # Quaternion -> rotation matrix (right-handed, column-major convention)
    # R derived from Hamilton quaternion with (x, y, z, w)
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz

    R = np.array([
        [1 - 2*(yy + zz),     2*(xy - wz),         2*(xz + wy)],
        [    2*(xy + wz), 1 - 2*(xx + zz),         2*(yz - wx)],
        [    2*(xz - wy),     2*(yz + wx),     1 - 2*(xx + yy)]
    ], dtype=np.float64)

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3,  3] = [px, py, pz]
    return T


def q_normalize(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([0.0, 0.0, 0.0, 1.0])

def q_multiply(q1, q2):
    # Both in [x, y, z, w]
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    return np.array([x, y, z, w])

def q_rotate_vec(q, v):
    # Rotate vector v by quaternion q([x,y,z,w])
    q = np.asarray(q, dtype=float)
    u = q[:3]
    s = q[3]
    v = np.asarray(v, dtype=float)
    return 2.0*np.dot(u, v)*u + (s*s - np.dot(u, u))*v + 2.0*s*np.cross(u, v)


def wxyz_to_xyzw(q_wxyz):
    q_wxyz = np.asarray(q_wxyz, dtype=float)
    return np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]], dtype=float)


def get_map(csv_file_path, color, lidar_bin_dir, frame_inc=10):
    # get all lines in csv for poses
    positions, quats = load_poses(csv_file_path)  # expect lists/arrays of equal length

    # get all binarized lidar in lidar dir (count all files)
    all_lidar_numbers = get_frame_numbers(lidar_bin_dir)

    map_points = []
    
    frame_min = 5000
    frame_max= 8000

    # iterate in lockstep, optionally skipping frames by frame_inc
    for i, (scan_num, pos, quat) in enumerate(zip(all_lidar_numbers, positions, quats)):
        if frame_inc is not None and frame_inc > 1 and (i % frame_inc) != 0:
            continue
        # if i > frame_max: #frame_inc is not None and frame_inc > 1 and (i % frame_inc) != 0:
        #     break

        # Load LiDAR points: expects Nx4 or Nx3 array; we use only xyz
        print(scan_num)
        points_np = load_lidar(os.path.join(lidar_bin_dir, f"lidar_pointcloud_{scan_num}.bin"))
        distances = np.linalg.norm(points_np, axis=1)
        mask = (distances < 40)
        points_np = points_np[mask]

        pc_xyz = points_np[:, :3].astype(np.float64, copy=False)

        # Build 4x4 pose matrix from pos & quat (assumes quat = [qx, qy, qz, qw])
        IMU_TO_LIDAR_T = np.array([-0.06286, 0.01557, 0.053345])
        IMU_TO_LIDAR_Q = q_normalize([0.0, 0.0, 1.0, 0.0])
        quat_fixed = wxyz_to_xyzw(quat)
        p_map = pos + q_rotate_vec(quat_fixed, IMU_TO_LIDAR_T)
        q_map = q_multiply(quat_fixed, IMU_TO_LIDAR_Q)

        transformation_matrix = pose_to_matrix(p_map, q_map)

        # Apply transform (homogeneous or direct form both fine)
        xyz_homogeneous = np.hstack([pc_xyz, np.ones((pc_xyz.shape[0], 1), dtype=np.float64)])
        
        transformed_xyz = (xyz_homogeneous @ transformation_matrix.T)[:, :3]

        map_points.append(transformed_xyz)

    map_points = np.vstack(map_points)  # (M, 3)

    o3d_pcd = o3d.geometry.PointCloud()
    o3d_pcd.points = o3d.utility.Vector3dVector(map_points)
    o3d_pcd.paint_uniform_color(color)
    pcd_downsampled = o3d_pcd.voxel_down_sample(voxel_size=1)

    return pcd_downsampled


def load_poses(csv_path):
    """Reads timestamp, x,y,z,qx,qy,qz,qw from CSV, skipping comments."""
    xs, ys, zs = [], [], []
    qxs, qys, qzs, qws = [], [], [], []
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith('#'):
                continue
            t, x, y, z, qx, qy, qz, qw = map(float, row)
            xs.append(x); ys.append(y); zs.append(z)
            qxs.append(qx); qys.append(qy); qzs.append(qz); qws.append(qw)
    points = np.vstack((xs, ys, zs)).T
    quats  = np.vstack((qws, qxs, qys, qzs)).T  # Open3D expects [w, x, y, z]
    return points, quats


def make_coordinate_frames(points, quats, stride=20, size=1.0):
    """
    Generate a list of small coordinate frames at every `stride`th pose.
    - `size` controls the axes length.
    """
    frames = []
    for i in range(0, len(points), stride):
        R = o3d.geometry.get_rotation_matrix_from_quaternion(quats[i])
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
        frame.rotate(R, center=(0,0,0))
        frame.translate(points[i])
        frames.append(frame)
    return frames


def get_lineset(csv_path, color):
    points, quats = load_poses(csv_path)

    # Create point cloud of positions
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Optionally color the trajectory (here: red line)
    colors = np.tile(color, (len(points), 1))
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # # # Coordinate frames every N poses
    # frames = make_coordinate_frames(points, quats, stride=100, size=2.0)

    # Draw trajectory as a line set
    lines = [[i, i+1] for i in range(len(points)-1)]
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines)
    )

    return pcd


def main():
    # environments = ["kittredge_loop", "main_campus"]
    environments = ["main_campus"]
    robots = [1, 2, 3]
    colors = [
        np.array([87, 227, 137])/255.0,
        np.array([192, 97, 203])/255.0,
        np.array([255, 163, 72])/255.0,
        np.array([98, 160, 234])/255.0,
    ]
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Robot Trajectory + Poses")

    for environment in environments:
        for robot, color in zip(robots, colors):
            csv_file_path = f"./solutions/{environment}_robot{robot}_ref.csv"
            lidar_bin_dir = f"/media/donceykong/donceys_data_ssd/datasets/CU_MULTI/data/{environment}/robot{robot}/lidar/original/pointclouds"

            line_set = get_lineset(csv_file_path, color)
            robot_map = get_map(csv_file_path, color, lidar_bin_dir)

            vis.add_geometry(robot_map)

    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()
