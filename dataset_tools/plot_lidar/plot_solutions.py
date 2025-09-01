#!/usr/bin/env python3
import sys
import csv
import numpy as np
import open3d as o3d

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
    environments = ["kittredge_loop", "main_campus"]
    robots = [1, 2, 3, 4]
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
            csv_file_path = f"{environment}_robot{robot}_ref.csv"
            line_set = get_lineset(csv_file_path, color)
            vis.add_geometry(line_set)

    vis.run()
    vis.destroy_window()

if __name__ == "__main__":
    main()
