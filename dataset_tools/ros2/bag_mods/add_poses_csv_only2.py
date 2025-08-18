#!/usr/bin/env python3
import os, csv, time
from collections import namedtuple

import numpy as np
import rclpy
from rclpy.serialization import serialize_message

from builtin_interfaces.msg import Time as BuiltinTime
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from tf2_msgs.msg import TFMessage

from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata

# add at the top
from decimal import Decimal, getcontext, ROUND_HALF_EVEN
getcontext().prec = 40
NSEC = Decimal("1000000000")
EPOCH_THRESH = Decimal("1000000000")  # ~2001+

def secs_to_ns_epoch_or_rel(t, t0):
    # Accept Decimal or str/float; normalize via Decimal for safety.
    t = Decimal(str(t))
    t0 = Decimal(str(t0))
    if t > EPOCH_THRESH:
        # Round to nearest ns; use ROUND_DOWN if you prefer monotonic non-increasing due to rounding.
        return int((t * NSEC).to_integral_value(rounding=ROUND_HALF_EVEN))
    return int(((t - t0) * NSEC).to_integral_value(rounding=ROUND_HALF_EVEN))

# -------------------- math helpers --------------------
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

def ns_to_time_msg(t_ns: int) -> BuiltinTime:
    time_msg = BuiltinTime(
        sec=int(t_ns // 1_000_000_000), 
        nanosec=int(t_ns % 1_000_000_000)
    )
    return time_msg

# def secs_to_ns_epoch_or_rel(t: float, t0: float) -> int:
#     # If timestamps look like UNIX epoch seconds, keep absolute
#     # else make them start at 0
#     if t > 1_000_000_000:   # ~2001+
#         return int(t * 1e9)
#     return int((t - t0) * 1e9)


# -------------------- CSV loader --------------------
# def load_csv_poses(path):
#     records = []
#     with open(path, newline="") as f:
#         reader = csv.reader(f)
#         for row in reader:
#             if not row or row[0].startswith("#"):
#                 continue
#             t = float(row[0])
#             pos = np.array(
#                 [float(row[1]), float(row[2]), float(row[3])]
#             )
#             quat = np.array(
#                 [float(row[4]), float(row[5]), float(row[6]), float(row[7])]
#             )
#             records.append(PoseRecord(t, pos, quat))
#     if not records:
#         raise RuntimeError(f"No pose rows found in {path}")
#     records.sort(key=lambda r: r.time)
#     times = np.array([r.time for r in records])
#     positions = np.stack([r.position for r in records])
#     quaternions = np.stack([q_normalize(r.orientation) for r in records])

#     # Optional: make first pose origin 
#     # (comment out if you want absolute coordinates)
#     origin = positions[0].copy()
#     positions = positions - origin

#     return times, positions, quaternions

def load_csv_poses(path):
    records = []
    with open(path, newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            t = Decimal(row[0])  # <-- keep full precision from CSV
            pos = np.array([float(row[1]), float(row[2]), float(row[3])])
            quat = np.array([float(row[4]), float(row[5]), float(row[6]), float(row[7])])
            records.append(PoseRecord(t, pos, quat))

    if not records:
        raise RuntimeError(f"No pose rows found in {path}")

    records.sort(key=lambda r: r.time)
    # times as a plain Python list of Decimals (avoid NumPy float coercion)
    times = [r.time for r in records]
    positions = np.stack([r.position for r in records])
    quaternions = np.stack([q_normalize(r.orientation) for r in records])

    # origin = positions[0].copy()
    # quaternions = 
    # positions = positions - origin

    return times, positions, quaternions


# for interpolation helper
PoseRecord = namedtuple("PoseRecord", ["time", "position", "orientation"])


# ==================== MAIN ====================
if __name__ == "__main__":
    rclpy.init()

    data_root = "/home/donceykong/Data/cu_multi"
    env = "main_campus"
    robots = [1]

    # IMU -> LiDAR extrinsic (IMU frame to LiDAR frame)
    IMU_TO_LIDAR_T = np.array([-0.06286, 0.01557, 0.053345])
    IMU_TO_LIDAR_Q = q_normalize([0.0, 0.0, 1.0, 0.0])

    for robot in robots:
        bag_dir = os.path.join(data_root, f"{env}/robot{robot}")
        in_uri = os.path.join(bag_dir, f"robot{robot}_{env}_poses")
        csv_file = os.path.join(bag_dir, f"{env}_robot{robot}_ref.csv")
        out_uri = in_uri + "_CSV_ONLY"

        # Topics
        odom_topic    = f"robot{robot}/lio_sam/mapping/odometry"
        path_topic    = f"robot{robot}/lio_sam/mapping/path"
        tf_topic      = "/tf"

        world_frame   = "world"
        map_frame     = f"robot{robot}_map"
        imu_frame     = f"robot{robot}_imu_link"
        lidar_frame   = f"robot{robot}_os_sensor"

        # --- load CSV poses once (IMU frame), then apply extrinsic to LiDAR frame ---
        csv_times, csv_pos_imu, csv_quat_imu, = load_csv_poses(csv_file)
        t0 = csv_times[0]

        # Init Writer
        writer = SequentialWriter()
        writer.open(
            StorageOptions(uri=out_uri, storage_id="sqlite3"),
            ConverterOptions(
                input_serialization_format="cdr", 
                output_serialization_format="cdr"
            ),
        )

        # Declare topics
        writer.create_topic(TopicMetadata(
            id=1,
            name=odom_topic, 
            type="nav_msgs/msg/Odometry", 
            serialization_format="cdr"
        ))
        writer.create_topic(TopicMetadata(
            id=2,
            name=path_topic, 
            type="nav_msgs/msg/Path", 
            serialization_format="cdr"
        ))
        writer.create_topic(TopicMetadata(
            id=3,
            name=tf_topic, 
            type="tf2_msgs/msg/TFMessage", 
            serialization_format="cdr"
        ))

        # Rolling path message
        path_msg = Path()
        path_msg.header.frame_id = world_frame
        p_lidar_origin = None
        q_lidar_origin = None
        for t_s, p_imu, q_imu in zip(csv_times, csv_pos_imu, csv_quat_imu):
            # Compose LiDAR pose in map:  
            # T_map_lidar = T_map_imu * T_imu_lidar
            p_lidar = p_imu + q_rotate_vec(q_imu, IMU_TO_LIDAR_T)
            q_lidar = q_multiply(q_imu, IMU_TO_LIDAR_Q)
            
            # Convert to lists
            imu_xyz = p_imu.tolist()
            imu_quat_xyzw = q_imu.tolist()
            imu2lidar_xyz = IMU_TO_LIDAR_T.tolist()
            imu2lidar_quat_xyzw = IMU_TO_LIDAR_Q.tolist()
            lidar_xyz = p_lidar.tolist()
            lidar_quat_xyzw = q_lidar.tolist()

            if p_lidar_origin is None and q_lidar_origin is None:
                p_lidar_origin = lidar_xyz
                q_lidar_origin = lidar_quat_xyzw

            # Timestamp in ns (epoch if CSV looks like epoch; else start at 0)
            stamp_ns = secs_to_ns_epoch_or_rel(t_s, t0)
            stamp_msg = ns_to_time_msg(stamp_ns)

            # --- Odometry (map -> lidar_frame) ---
            # leave covariance, twist as zeros
            odom = Odometry()
            odom.header.frame_id = world_frame
            odom.child_frame_id = imu_frame
            odom.header.stamp = stamp_msg
            
            odom.pose.pose.position.x = lidar_xyz[0]
            odom.pose.pose.position.y = lidar_xyz[1]
            odom.pose.pose.position.z = lidar_xyz[2]
            odom.pose.pose.orientation.x = lidar_quat_xyzw[0]
            odom.pose.pose.orientation.y = lidar_quat_xyzw[1]
            odom.pose.pose.orientation.z = lidar_quat_xyzw[2]
            odom.pose.pose.orientation.w = lidar_quat_xyzw[3]
            
            writer.write(odom_topic, serialize_message(odom), stamp_ns)

            # --- Path (in map frame, LiDAR poses) ---
            ps = PoseStamped()
            ps.header.frame_id = world_frame
            ps.header.stamp = stamp_msg
            ps.pose.position.x = lidar_xyz[0]
            ps.pose.position.y = lidar_xyz[1]
            ps.pose.position.z = lidar_xyz[2]
            ps.pose.orientation.x = lidar_quat_xyzw[0]
            ps.pose.orientation.y = lidar_quat_xyzw[1]
            ps.pose.orientation.z = lidar_quat_xyzw[2]
            ps.pose.orientation.w = lidar_quat_xyzw[3]

            path_msg.poses.append(ps)
            path_msg.header.stamp = stamp_msg
            
            writer.write(path_topic, serialize_message(path_msg), stamp_ns)

            # --- TF ---
            # world -> map (static)
            tf_world_map = TransformStamped()
            tf_world_map.header.frame_id = world_frame
            tf_world_map.child_frame_id = map_frame
            tf_world_map.header.stamp = stamp_msg
            tf_world_map.transform.translation.x = p_lidar_origin[0]
            tf_world_map.transform.translation.y = p_lidar_origin[1]
            tf_world_map.transform.translation.z = p_lidar_origin[2]
            tf_world_map.transform.rotation.x = q_lidar_origin[0]
            tf_world_map.transform.rotation.y = q_lidar_origin[1]
            tf_world_map.transform.rotation.z = q_lidar_origin[2]
            tf_world_map.transform.rotation.w = q_lidar_origin[3]

            # world -> imu (dynamic)
            tf_world_imu = TransformStamped()
            tf_world_imu.header.frame_id = world_frame
            tf_world_imu.child_frame_id = imu_frame
            tf_world_imu.header.stamp = stamp_msg
            tf_world_imu.transform.translation.x = imu_xyz[0]
            tf_world_imu.transform.translation.y = imu_xyz[1]
            tf_world_imu.transform.translation.z = imu_xyz[2]
            tf_world_imu.transform.rotation.x = imu_quat_xyzw[0]
            tf_world_imu.transform.rotation.y = imu_quat_xyzw[1]
            tf_world_imu.transform.rotation.z = imu_quat_xyzw[2]
            tf_world_imu.transform.rotation.w = imu_quat_xyzw[3]

            # imu -> lidar (static, repeated on /tf for simplicity)
            tf_imu_lidar = TransformStamped()
            tf_imu_lidar.header.frame_id = imu_frame
            tf_imu_lidar.child_frame_id = lidar_frame
            tf_imu_lidar.header.stamp = stamp_msg
            tf_imu_lidar.transform.translation.x = imu2lidar_xyz[0]
            tf_imu_lidar.transform.translation.y = imu2lidar_xyz[1]
            tf_imu_lidar.transform.translation.z = imu2lidar_xyz[2]
            tf_imu_lidar.transform.rotation.x = imu2lidar_quat_xyzw[0]
            tf_imu_lidar.transform.rotation.y = imu2lidar_quat_xyzw[1]
            tf_imu_lidar.transform.rotation.z = imu2lidar_quat_xyzw[2]
            tf_imu_lidar.transform.rotation.w = imu2lidar_quat_xyzw[3]

            tf_msg = TFMessage(transforms=[tf_world_map, tf_world_imu, tf_imu_lidar])
            writer.write(tf_topic, serialize_message(tf_msg), stamp_ns)

        # Done
        try:
            writer.close()
        except Exception:
            # Some distros auto-close; ignore if close() doesn't exist
            pass

        print(f"Wrote bag at: {out_uri}")
