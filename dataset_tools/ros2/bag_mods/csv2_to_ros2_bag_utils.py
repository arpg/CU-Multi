import os
import csv
import numpy as np
from collections import namedtuple
from tqdm import tqdm

import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import (
    SequentialReader,
    SequentialWriter,
    StorageOptions,
    ConverterOptions,
    TopicMetadata,
)

import open3d as o3d

# for interpolation helper
PoseRecord = namedtuple("PoseRecord", ["time", "position", "orientation"])

# -------------------- Math utils --------------------
def quat_to_rot(q):
    x, y, z, w = q
    n = np.linalg.norm(q)
    if n == 0: return np.eye(3)
    x, y, z, w = x/n, y/n, z/n, w/n
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return np.array([
        [1-2*(yy+zz), 2*(xy-wz), 2*(xz+wy)],
        [2*(xy+wz), 1-2*(xx+zz), 2*(yz-wx)],
        [2*(xz-wy), 2*(yz+wx), 1-2*(xx+yy)],
    ])

def rot_to_quat(R):
    """Convert rotation matrix to quaternion [x, y, z, w]."""
    m = R
    tr = m[0,0] + m[1,1] + m[2,2]
    if tr > 0:
        S = np.sqrt(tr+1.0) * 2
        w = 0.25 * S
        x = (m[2,1] - m[1,2]) / S
        y = (m[0,2] - m[2,0]) / S
        z = (m[1,0] - m[0,1]) / S
    elif (m[0,0] > m[1,1]) and (m[0,0] > m[2,2]):
        S = np.sqrt(1.0 + m[0,0] - m[1,1] - m[2,2]) * 2
        w = (m[2,1] - m[1,2]) / S
        x = 0.25 * S
        y = (m[0,1] + m[1,0]) / S
        z = (m[0,2] + m[2,0]) / S
    elif m[1,1] > m[2,2]:
        S = np.sqrt(1.0 + m[1,1] - m[0,0] - m[2,2]) * 2
        w = (m[0,2] - m[2,0]) / S
        x = (m[0,1] + m[1,0]) / S
        y = 0.25 * S
        z = (m[1,2] + m[2,1]) / S
    else:
        S = np.sqrt(1.0 + m[2,2] - m[0,0] - m[1,1]) * 2
        w = (m[1,0] - m[0,1]) / S
        x = (m[0,2] + m[2,0]) / S
        y = (m[1,2] + m[2,1]) / S
        z = 0.25 * S
    q = np.array([x, y, z, w])
    # normalize for safety
    q /= np.linalg.norm(q)
    return q

# ++++++++++++++ O=O=O=O=O=O=O=O ++++++++++++++
def apply_tf_to_pose_array(tf_mat, positions, quaternions):
    """
    Left-multiply each pose by tf_mat: R' = R_tf R, p' = R_tf p + t_tf.
    """
    R_tf = tf_mat[:3, :3]
    t_tf = tf_mat[:3, 3]
    new_pos = np.empty_like(positions)
    new_quat = np.empty_like(quaternions)
    for i, (p, q) in enumerate(zip(positions, quaternions)):
        R = quat_to_rot(q)
        R_out = R @ R_tf
        p_out = p + t_tf
        new_pos[i] = p_out
        new_quat[i] = rot_to_quat(R_out)
    
    return new_pos, new_quat

# -------------------- CSV + interpolation --------------------
def load_csv_poses(path):
    records = []
    with open(path, newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            t = float(row[0])
            pos = np.array([float(row[1]), float(row[2]), float(row[3])])
            quat = np.array([float(row[4]), float(row[5]), float(row[6]), float(row[7])])
            records.append(PoseRecord(t, pos, quat))
    if not records:
        raise RuntimeError(f"No pose rows found in {path}")
    records.sort(key=lambda r: r.time)
    times = np.array([r.time for r in records])
    positions = np.stack([r.position for r in records])
    quaternions = np.stack([r.orientation for r in records])

    # Optional: make first pose origin (comment out if you want absolute coordinates)
    origin = positions[0].copy()
    positions = positions - origin

    return times, positions, quaternions

def slerp(q0, q1, alpha):
    """Spherical linear interpolation of two quaternions."""
    dot = np.dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        out = q0 + alpha * (q1 - q0)
        return out / np.linalg.norm(out)
    theta_0 = np.arccos(np.clip(dot, -1.0, 1.0))
    sin_0 = np.sin(theta_0)
    theta = theta_0 * alpha
    sin_t = np.sin(theta)
    s0 = np.cos(theta) - dot * sin_t / sin_0
    s1 = sin_t / sin_0
    out = (s0 * q0) + (s1 * q1)
    return out / np.linalg.norm(out)

def get_pose_at(time_msg, csv_times, csv_pos, csv_quat):
    """
    Return: 
        - Interpolated (pos, quat) for a given: builtin_interfaces.msg.Time()
        - 
    """
    t_sec = time_msg.sec + time_msg.nanosec * 1e-9
    idx = np.searchsorted(csv_times, t_sec)

    # exact match
    if idx < len(csv_times) and abs(csv_times[idx] - t_sec) < 1e-9:
        return csv_pos[idx], csv_quat[idx]
    # out of CSV range
    if idx == 0 or idx >= len(csv_times):
        raise ValueError(f"Timestamp {t_sec:.9f} outside CSV range")
    
    # interpolate
    t0, t1 = csv_times[idx - 1], csv_times[idx]
    alpha = (t_sec - t0) / (t1 - t0)
    p0, p1 = csv_pos[idx - 1], csv_pos[idx]
    q0, q1 = csv_quat[idx - 1], csv_quat[idx]
    pos = p0 + alpha * (p1 - p0)
    quat = slerp(q0, q1, alpha)
    return pos, quat

# -------------------- Open3D viz helper --------------------
def visualize_open3d(points_xyz, every=1):
    """Open3D viewer of the trajectory: red points + green connecting lines."""
    if points_xyz.shape[0] == 0:
        print("[viz] No points to visualize.")
        return
    
    # Optional thinning
    idx = np.arange(0, points_xyz.shape[0], max(1, every))
    P = points_xyz[idx]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(P)
    pcd.paint_uniform_color([1, 0, 0])

    lines = [[i, i+1] for i in range(len(P)-1)]
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(P),
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.colors = o3d.utility.Vector3dVector([[0, 1, 0] for _ in lines])  # green
    o3d.visualization.draw_geometries([pcd, line_set])

# ==================== MAIN ====================
if __name__ == "__main__":
    rclpy.init()

    data_root = "/media/donceykong/doncey_ssd_02/datasets/CU_MULTI/ros1_bags/"
    env = "main_campus"
    robots = [2]

    # IMU -> LiDAR extrinsic as homogeneous transform
    extrinsic_matrix = np.array([
        [-1.0,  0.0, 0.0, -0.058038],
        [ 0.0, -1.0, 0.0,  0.015573],
        [ 0.0,  0.0, 1.0,  0.049603],
        [ 0.0,  0.0, 0.0,  1.000000]
    ], dtype=float)

    # Toggle quick visualization
    VISUALIZE = False
    VIZ_DOWNSAMPLE_EVERY = 5  # plot every Nth point to keep it snappy

    for robot in robots:
        bag_dir = os.path.join(data_root, f"{env}/robot{robot}")
        in_uri = os.path.join(bag_dir, f"robot{robot}_{env}_poses")
        csv_file = os.path.join(bag_dir, f"{env}_robot{robot}_ref.csv")
        out_uri = in_uri + "_UTM_DEL"

        # --- load CSV poses once (IMU frame), then apply extrinsic to LiDAR frame ---
        csv_times, csv_pos, csv_quat = load_csv_poses(csv_file)
        csv_pos, csv_quat = apply_tf_to_pose_array(extrinsic_matrix, csv_pos, csv_quat)

        # quick Open3D view (optional)
        if VISUALIZE:
            visualize_open3d(csv_pos, every=VIZ_DOWNSAMPLE_EVERY)

        # --- open reader ---
        reader = SequentialReader()
        reader.open(
            StorageOptions(uri=in_uri, storage_id="sqlite3"),
            ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr",
            ),
        )
        topics_meta = reader.get_all_topics_and_types()

        # --- prepare writer with same topics ---
        writer = SequentialWriter()
        writer.open(
            StorageOptions(uri=out_uri, storage_id="sqlite3"),
            ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr",
            ),
        )
        for tm in topics_meta:
            writer.create_topic(tm)

        # map topic → ROS message class
        msg_types = {tm.name: get_message(tm.type) for tm in topics_meta}

        odom_topic    = f"robot{robot}/lio_sam/mapping/odometry"
        path_topic    = f"robot{robot}/lio_sam/mapping/path"
        tf_topic      = "/tf"
        odom_frame_id = f"robot{robot}_base_link"

        # --- process all records ---
        pbar = tqdm(desc=f"robot{robot}", unit="msg")

        while reader.has_next():
            topic, data, t = reader.read_next()
            MsgType = msg_types[topic]
            msg = deserialize_message(data, MsgType)

            if topic == odom_topic:
                print(f"{odom_topic} child_frame_id: {msg.child_frame_id}")
                print(f"{odom_topic} Header: {msg.header}")
                # capture child_frame_id on first odom if needed
                if odom_frame_id is None:
                    odom_frame_id = msg.child_frame_id

                pos, quat = get_pose_at(msg.header.stamp, csv_times, csv_pos, csv_quat)

                msg.pose.pose.position.x = float(pos[0])
                msg.pose.pose.position.y = float(pos[1])
                msg.pose.pose.position.z = float(pos[2])
                msg.pose.pose.orientation.x = float(quat[0])
                msg.pose.pose.orientation.y = float(quat[1])
                msg.pose.pose.orientation.z = float(quat[2])
                msg.pose.pose.orientation.w = float(quat[3])
                out_data = serialize_message(msg)

            elif topic == tf_topic and odom_frame_id is not None:
                # patch only the odom→base_link (or whatever your odom_frame_id is)
                for tf_st in msg.transforms:
                    if tf_st.child_frame_id == odom_frame_id:
                        pos, quat = get_pose_at(tf_st.header.stamp, csv_times, csv_pos, csv_quat)
                        tf_st.transform.translation.x = float(pos[0])
                        tf_st.transform.translation.y = float(pos[1])
                        tf_st.transform.translation.z = float(pos[2])
                        tf_st.transform.rotation.x = float(quat[0])
                        tf_st.transform.rotation.y = float(quat[1])
                        tf_st.transform.rotation.z = float(quat[2])
                        tf_st.transform.rotation.w = float(quat[3])
                out_data = serialize_message(msg)

            elif topic == path_topic:
                # update each stamped pose in the Path
                for pose_stamped in msg.poses:
                    pos, quat = get_pose_at(pose_stamped.header.stamp, csv_times, csv_pos, csv_quat)
                    pose_stamped.pose.position.x = float(pos[0])
                    pose_stamped.pose.position.y = float(pos[1])
                    pose_stamped.pose.position.z = float(pos[2])
                    pose_stamped.pose.orientation.x = float(quat[0])
                    pose_stamped.pose.orientation.y = float(quat[1])
                    pose_stamped.pose.orientation.z = float(quat[2])
                    pose_stamped.pose.orientation.w = float(quat[3])
                out_data = serialize_message(msg)

            else:
                # everything else untouched
                out_data = data

            writer.write(topic, out_data, t)
            pbar.update(1)

        pbar.close()
        reader.close()
        writer.close()

    rclpy.shutdown()
