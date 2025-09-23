import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from nav_msgs.msg import Path
import os
import csv
from decimal import Decimal, getcontext
import math
from bisect import bisect_right

getcontext().prec = 40
NSEC = Decimal("1000000000")


# ---------- Formatting helpers ----------
def fmt(v, places=10, strip=False):
    s = f"{float(v):.{places}f}"
    return s.rstrip('0').rstrip('.') if strip else s


def rostime_to_sec(sec: int, nanosec: int) -> Decimal:
    return Decimal(sec) + (Decimal(nanosec) / NSEC)

# ---------- ROS bag helpers ----------
def open_reader(bag_path: str):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def get_last_message(bag_path, topic_name):
    reader = open_reader(bag_path)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    if topic_name not in type_map:
        print(f"[get_last_message] Topic '{topic_name}' not found in {bag_path}")
        return None, None

    msg_type = get_message(type_map[topic_name])
    last_msg, last_stamp = None, None

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == topic_name:
            last_msg = deserialize_message(data, msg_type)
            last_stamp = t  # nanoseconds since epoch (int)
    return last_stamp, last_msg


def get_pointcloud_stamps(lidar_bag_path: str, topic: str):
    """
    Collect timestamps for all messages on `topic` from `lidar_bag_path`.

    Returns:
        List[Decimal]: sorted timestamps in seconds (Decimal), preferring
        message header time; falls back to bag time if needed.
    """
    reader = open_reader(lidar_bag_path)

    # Build topic -> type map and validate the topic exists
    topics = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics}
    if topic not in type_map:
        print(f"[get_pointcloud_stamps] Topic '{topic}' not found in {lidar_bag_path}")
        return []

    # Load the message class (e.g., sensor_msgs/msg/PointCloud2)
    msg_cls = get_message(type_map[topic])

    stamps = []
    while reader.has_next():
        t_name, data, t_ns = reader.read_next()
        if t_name == topic:
            msg = deserialize_message(data, msg_cls)
            # Prefer sensor timestamp from header
            try:
                ts = rostime_to_sec(msg.header.stamp.sec, msg.header.stamp.nanosec)
            except Exception:
                # Fallback to bag-time if header missing
                ts = Decimal(t_ns) / NSEC
            stamps.append(ts)

    stamps.sort()
    return stamps


# ---------- Quaternion math (w,x,y,z? -> geometry_msgs uses x,y,z,w) ----------
def quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x/n, y/n, z/n, w/n)


def quat_dot(q0, q1):
    return q0[0]*q1[0] + q0[1]*q1[1] + q0[2]*q1[2] + q0[3]*q1[3]


def quat_slerp(q0, q1, u: float):
    """Spherical linear interpolation for geometry_msgs quaternions (x,y,z,w). u in [0,1]."""
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)
    dot = quat_dot(q0, q1)

    # Take shortest path
    if dot < 0.0:
        q1 = (-q1[0], -q1[1], -q1[2], -q1[3])
        dot = -dot

    DOT_THRESHOLD = 0.9995
    if dot > DOT_THRESHOLD:
        # Very close: fall back to lerp + normalize
        x = q0[0] + u*(q1[0]-q0[0])
        y = q0[1] + u*(q1[1]-q0[1])
        z = q0[2] + u*(q1[2]-q0[2])
        w = q0[3] + u*(q1[3]-q0[3])
        return quat_normalize((x,y,z,w))

    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * u
    sin_theta = math.sin(theta)

    s0 = math.sin(theta_0 - theta) / sin_theta_0
    s1 = sin_theta / sin_theta_0

    x = (s0*q0[0]) + (s1*q1[0])
    y = (s0*q0[1]) + (s1*q1[1])
    z = (s0*q0[2]) + (s1*q1[2])
    w = (s0*q0[3]) + (s1*q1[3])
    return (x,y,z,w)


# ---------- Interpolation over a Path ----------
def path_to_arrays(path_msg: Path):
    """Extract arrays of times (Decimal), positions (floats), quats (x,y,z,w floats)."""
    T = []
    P = []  # (x,y,z)
    Q = []  # (x,y,z,w)
    for ps in path_msg.poses:
        T.append(rostime_to_sec(ps.header.stamp.sec, ps.header.stamp.nanosec))
        pos = ps.pose.position
        P.append((float(pos.x), float(pos.y), float(pos.z)))
        q = ps.pose.orientation
        Q.append((float(q.x), float(q.y), float(q.z), float(q.w)))
    return T, P, Q


def interp_pose_at_time(T, P, Q, t: Decimal, clamp=False):
    """
    Interpolate pose at time t.
    - T: sorted list of Decimal times
    - P: positions [(x,y,z)]
    - Q: quats [(x,y,z,w)]
    Returns (pos_tuple, quat_tuple) or None if out-of-range and clamp=False.
    """
    if not T:
        return None
    # out-of-range handling
    if t < T[0]:
        if not clamp: return None
        return P[0], Q[0]
    if t > T[-1]:
        if not clamp: return None
        return P[-1], Q[-1]

    # find right index (first T[idx] > t)
    idx = bisect_right(T, t)
    if idx == 0:
        return P[0], Q[0]
    if idx >= len(T):
        return P[-1], Q[-1]

    t0, t1 = T[idx-1], T[idx]
    p0, p1 = P[idx-1], P[idx]
    q0, q1 = Q[idx-1], Q[idx]

    # fraction in [0,1]
    denom = float((t1 - t0) / (t1 - t0)) if t1 != t0 else 1.0  # safe guard
    u = float((t - t0) / (t1 - t0)) if t1 != t0 else 0.0

    # linear interp position
    x = p0[0] + u*(p1[0]-p0[0])
    y = p0[1] + u*(p1[1]-p0[1])
    z = p0[2] + u*(p1[2]-p0[2])

    # slerp orientation
    qx, qy, qz, qw = quat_slerp(q0, q1, u)

    return (x, y, z), (qx, qy, qz, qw)


# ---------- CSV ----------
def ensure_csv_with_header(csv_path: str, header):
    exists = os.path.exists(csv_path)
    if not exists or os.path.getsize(csv_path) == 0:
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(header)



if __name__ == "__main__":
    # --- user config ---
    data_root = "/home/donceykong/Data/cu_multi"
    env = "main_campus"
    robot_nums = [1]
    # CLAMP_INSIDE = True    # set True to fill in poses using first posse instead of skipping
    CLAMP_OUTSIDE = True    # set True to clamp to end poses instead of skipping
    # -------------------

    for robot_num in robot_nums:
        bag_dir = os.path.join(data_root, f"{env}/robot{robot_num}")
        poses_bag_path = os.path.join(bag_dir, f"robot{robot_num}_{env}_poses")
        lidar_bag_path = os.path.join(bag_dir, f"robot{robot_num}_{env}_lidar")
        lidar_topic = f"robot{robot_num}/ouster/points"
        path_topic = f"robot{robot_num}/lio_sam/mapping/path"

        # 1) load the last Path
        _, path_msg = get_last_message(poses_bag_path, path_topic)
        if path_msg is None or len(path_msg.poses) == 0:
            print(f"[robot{robot_num}] No Path found on '{path_topic}'")
            continue

        T, P, Q = path_to_arrays(path_msg)
        print(f"[robot{robot_num}] Loaded path with {len(T)} poses "
              f"from {T[0]} to {T[-1]} sec")

        # 2) collect LiDAR scan timestamps
        lidar_stamps = get_pointcloud_stamps(lidar_bag_path, lidar_topic)

        out_csv = os.path.join(
            bag_dir,
            f"robot{robot_num}_{env}_poses_at_lidar_scans__{lidar_topic.replace('/','_')}.csv"
        )
        ensure_csv_with_header(
            out_csv,
            ["scan_timestamp_sec", "x", "y", "z", "qx", "qy", "qz", "qw"]
        )

        wrote = 0
        with open(out_csv, "a", newline="") as f:
            w = csv.writer(f)
            for ts in lidar_stamps:
                res = interp_pose_at_time(T, P, Q, ts, clamp=CLAMP_OUTSIDE)
                if res is None:
                    # out of range, skipping
                    continue
                (x, y, z), (qx, qy, qz, qw) = res
                w.writerow([
                    str(ts),
                    fmt(x), fmt(y), fmt(z),
                    fmt(qx), fmt(qy), fmt(qz), fmt(qw),
                ])
                wrote += 1

        print(f"[robot{robot_num}] {lidar_topic}: wrote {wrote} rows -> {out_csv}")
