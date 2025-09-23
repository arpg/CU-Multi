#!/usr/bin/env python3
"""
"""
import shutil
import re
from typing import Tuple, Optional

import csv, os, sys
from typing import Dict, Tuple, Optional, Any
from decimal import Decimal, getcontext
try:
    import rclpy
    from builtin_interfaces.msg import Time as Ros2Time
    from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import deserialize_message, serialize_message
except Exception as e:
    print("ERROR: Requires ROS 2 (rclpy, rosbag2_py, rosidl_runtime_py). Source your ROS 2 env.", file=sys.stderr)
    raise

# ---------- USER VARIABLES (EDIT THESE) ----------
# IN_BAG_DIR      = "input_ros2_bag"           # folder containing rosbag2 (metadata.yaml, *.db3)
# OUT_BAG_DIR     = "output_ros2_bag_clipped"  # output folder to create
# WINDOWS_CSV     = "robot_windows_unix.csv"

KEEP_NONROBOT   = True     # keep & clip non-robot topics to the global window (no shift)
NAMESPACE_DEPTH = 1        # '/robot1/cam' -> 'robot1'
# REWRITE_HEADERS = True
VERBOSE         = True

# ALWAYS_KEEP_TOPICS = ("ouster/metadata",)  # keep as-is; no time adjust

# -------------------------------------------------

# High precision for UNIX seconds math
getcontext().prec = 30

DecimalPair = Tuple[Decimal, Decimal]

def load_windows_csv(path: str) -> Dict[str, DecimalPair]:
    out: Dict[str, DecimalPair] = {}
    with open(path, "r", newline="") as f:
        csv_rdr = csv.DictReader(f)
        headers = {h.strip().lower(): h for h in (csv_rdr.fieldnames or [])}

        robot_names = headers.get("robot")
        start_times = headers.get("unix_start_ts")
        end_times = headers.get("unix_end_ts")

        if not (robot_name and start_times and end_times):
            raise ValueError("CSV must have columns: robot, unix_start_ts, unix_end_ts")

        for row in csv_rdr:
            robot_name = str(row[robot_names]).strip()
            start_time = Decimal(row[start_times].strip())
            end_time = Decimal(row[end_times].strip())
            out[robot_name] = (start_time, end_time)
    return out

def topic_robot(topic: str, depth: int) -> Optional[str]:
    parts = [p for p in topic.split('/') if p]
    if len(parts) < depth:
        return None
    if depth == 1:
        return parts[0]
    return "_".join(parts[:depth])

def fmt9(x: Decimal) -> str:
    return format(x, ".9f")


def topic_matches(name: str, suffixes) -> bool:
    name = name.strip("/")
    for s in suffixes:
        s = s.strip("/")
        if name == s or name.endswith("/" + s):
            return True
    return False

# ---- Time conversions ----
NS_IN_S = Decimal(1_000_000_000)

def decimal_from_rosbag2_timestamp_ns(ns: int) -> Decimal:
    # rosbag2 message timestamp (reader.read_next()[2]) is an int nanoseconds since epoch
    return Decimal(ns) / NS_IN_S

def rosbag2_timestamp_ns_from_decimal(t: Decimal) -> int:
    # clamp to integer nanoseconds
    return int((t * NS_IN_S).to_integral_value(rounding="ROUND_HALF_EVEN"))

def decimal_from_ros2_time(t: Ros2Time) -> Decimal:
    return Decimal(t.sec) + (Decimal(t.nanosec) / NS_IN_S)

def to_ros2_time_decimal(t: Decimal) -> Ros2Time:
    secs = int(t // 1)  # floor
    frac = t - Decimal(secs)
    if frac < 0:
        secs -= 1
        frac += 1
    nsec = int((frac * NS_IN_S).to_integral_value(rounding="ROUND_HALF_EVEN"))
    if nsec >= 1_000_000_000:
        secs += 1
        nsec -= 1_000_000_000
    msg = Ros2Time()
    msg.sec = int(secs)
    msg.nanosec = int(nsec)
    return msg

# ---- Header/TF rewriting ----
def try_shift_header_stamp(msg: Any, dt_shift: Decimal):
    try:
        hdr = getattr(msg, "header", None)
        if hdr is None:
            return
        st = getattr(hdr, "stamp", None)
        if st is None:
            return
        curr = decimal_from_ros2_time(st)
        new_t = curr - dt_shift
        setattr(hdr, "stamp", to_ros2_time_decimal(new_t))
    except Exception:
        pass


def try_shift_tf_stamps(msg: Any, dt_shift: Decimal):
    # tf2_msgs/msg/TFMessage has transforms: list[geometry_msgs/msg/TransformStamped]
    try:
        transforms = getattr(msg, "transforms", None)
        if not transforms:
            return
        for ts in transforms:
            st = getattr(ts.header, "stamp", None)
            if st is None:
                continue
            curr = decimal_from_ros2_time(st)
            new_t = curr - dt_shift
            ts.header.stamp = to_ros2_time_decimal(new_t)
    except Exception:
        pass


def clip_and_shift_nav_path_poses(msg: Any,
                                  lo: Decimal,
                                  hi: Decimal,
                                  dt_shift: Decimal) -> bool:
    """
    """
    try:
        poses = getattr(msg, "poses", None)
        kept = []
        for ps in poses:
            h = getattr(ps, "header", None)
            st = getattr(h, "stamp", None)
            t = Decimal(st.sec) + (Decimal(st.nanosec) / NS_IN_S)
            if lo <= t <= hi:
                new_t = t - dt_shift
                ps.header.stamp = to_ros2_time_decimal(new_t)
                kept.append(ps)

        msg.poses = kept
        return len(kept) > 0
    except Exception:
        print(f"\n\nPOSE FIX EXCEPTION CALLED!\n\n")
        # If anything goes weird, keep the message rather than nuking it.
        return True


def parse_env_and_robot(s: str) -> Tuple[str, Optional[str]]:
    """
    Parse names like "main_campus_r1" or "north_site_robot12" (underscores or hyphens).
    Returns (env, robot_name) where robot_name is normalized as "robot<digits>".

    Examples:
      "main_campus_r1"      -> ("main_campus", "robot1")
      "alpha-beta_robot07"  -> ("alpha-beta", "robot7")
      "sim_world"           -> ("sim_world", None)
    """
    # Split on underscores or hyphens, inspect the last token
    parts = re.split(r"[_-]", s)
    last = parts[-1] if parts else s

    m = re.fullmatch(r"(?:robot)?r?(\d+)", last, flags=re.IGNORECASE)
    if m:
        num = int(m.group(1))          # normalize leading zeros -> 7 not 07
        env = "_".join(parts[:-1]) if len(parts) > 1 else ""
        robot_name = f"robot{num}"
        return env, robot_name
    else:
        # No robot suffix; whole string is env
        return s, None


# ---- Core helper: clip a single rosbag2 directory by window, shift to global_start ----
def clip_ros2_bag_dir(in_dir: str,
                      out_dir: str,
                      id_int: int,
                      lo: Decimal,
                      hi: Decimal,
                      global_start: Decimal,
                      *,
                      always_keep_topics = ("ouster/metadata","tf_static",),
                      verbose: bool = True) -> None:
    """
    """
    NS_IN_S = Decimal(1_000_000_000)

    def dec_from_ns(ns: int) -> Decimal:
        return Decimal(ns) / NS_IN_S

    def ns_from_dec(t: Decimal) -> int:
        return int((t * NS_IN_S).to_integral_value(rounding="ROUND_HALF_EVEN"))

    def to_ros2_time_decimal(t: Decimal) -> Ros2Time:
        secs = int(t // 1)
        frac = t - Decimal(secs)
        if frac < 0:
            secs -= 1
            frac += 1
        nsec = int((frac * NS_IN_S).to_integral_value(rounding="ROUND_HALF_EVEN"))
        if nsec >= 1_000_000_000:
            secs += 1
            nsec -= 1_000_000_000
        out = Ros2Time()
        out.sec = int(secs)
        out.nanosec = int(nsec)
        return out

    def topic_matches(name: str, suffixes) -> bool:
        name = name.strip("/")
        for s in suffixes:
            s = s.strip("/")
            if name == s or name.endswith("/" + s):
                return True
        return False

    def try_shift_header(msg, dt: Decimal):
        try:
            hdr = getattr(msg, "header", None)
            if hdr is None or getattr(hdr, "stamp", None) is None:
                return
            
            # stamp := builtin_interfaces/Time
            curr = Decimal(hdr.stamp.sec) + Decimal(hdr.stamp.nanosec) / NS_IN_S
            new_t = curr - dt
            hdr.stamp = to_ros2_time_decimal(new_t)
        except Exception:
            pass

    def try_shift_tf(msg, dt: Decimal):
        try:
            transforms = getattr(msg, "transforms", None)
            if not transforms:
                return
            
            for ts in transforms:
                st = ts.header.stamp
                curr = Decimal(st.sec) + Decimal(st.nanosec) / NS_IN_S
                ts.header.stamp = to_ros2_time_decimal(curr - dt)
        except Exception:
            print(f"SHIFTING TF HEADER DIDNT WORK")
            pass

    # Open reader
    storage_in = StorageOptions(uri=in_dir, storage_id='sqlite3')
    converter = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage_in, converter)
    topics_and_types = reader.get_all_topics_and_types()

    # Prepare writer (recreate out_dir)
    if os.path.exists(out_dir):
        shutil.rmtree(out_dir)

    # Create Writer
    storage_out = StorageOptions(uri=out_dir, storage_id='sqlite3')
    writer = SequentialWriter()
    writer.open(storage_out, converter)

    # Register topics
    topic_type_map = {}
    for md in topics_and_types:
        writer.create_topic(TopicMetadata(
            # id = 0,
            name=md.name, 
            type=md.type,
            serialization_format='cdr',
            offered_qos_profiles=md.offered_qos_profiles,
        ))
        try:
            topic_type_map[md.name] = (md.type, get_message(md.type))
        except Exception:
            topic_type_map[md.name] = (md.type, None)

    dt_shift = (lo - global_start)

    total_in = total_out = 0
    wrote_meta_for = set()

    first_points_ts = None
    ouster_metadata_ts = None

    while reader.has_next():
        topic, serialized, t_ns = reader.read_next()
        total_in += 1
        t_dec = dec_from_ns(t_ns)

        out_time = t_dec - dt_shift
        out_ns = ns_from_dec(out_time)

        topic_stripped = topic.strip("/")
        if topic_stripped.endswith("/ouster/metadata"):
            print(f"\n\n {topic} at {out_ns}. Originally was at ts: {t_ns}.")
            ouster_metadata_ts = t_ns
        elif topic_stripped.endswith("/ouster/points"):
            if first_points_ts is None:
                print(f"\n\n {topic} at {out_ns}. Originally was at ts: {t_ns}.")
                first_points_ts = t_ns

        # if ouster_metadata_ts is not None and first_points_ts is not None:
            # print(f"\n first points ts: {first_points_ts}, first metadata ts: {ouster_metadata_ts}")
            
        # always-keep
        if topic_matches(topic, always_keep_topics):
            print(f"\n\nname: {topic}")
            if topic not in wrote_meta_for:
                out_ns = ns_from_dec(lo)
                writer.write(topic, serialized, out_ns)
                wrote_meta_for.add(topic)

                if verbose:
                    print(f"\n\n[keep-meta] {topic} at {out_ns}. Originally was at ts: {t_ns}.")
            continue

        if not (lo <= t_dec <= hi):
            continue

        _, type_cls = topic_type_map.get(topic, (None, None))
        # topic_stripped = topic.strip("/")
        if topic_stripped.endswith("/lio_sam/mapping/path"):
            # print(f"\n\nTOPIC IS PATH: {topic}")
            msg = deserialize_message(serialized, type_cls)
            try_shift_header(msg, dt_shift)
            try_shift_tf(msg, dt_shift)
            clip_and_shift_nav_path_poses(msg, lo, hi, dt_shift)
            new_bytes = serialize_message(msg)
        else:
            try:
                msg = deserialize_message(serialized, type_cls)
                try_shift_header(msg, dt_shift)
                try_shift_tf(msg, dt_shift)
                new_bytes = serialize_message(msg)
            except Exception:
                new_bytes = serialized
                print(f"\n\nEXCEPTION CALLED!\n\n")

        writer.write(topic, new_bytes, out_ns)
        total_out += 1

    if verbose:
        print(f"\n\nWrote {total_out}/{total_in} msgs: {in_dir} -> {out_dir}")


def clip_robot_bags(robot_time_windows):
    """ Shift each robot lidar and pose bag
    """
    DATA_ROOT = os.getenv("CU_MULTI_ROOT")
    if DATA_ROOT is None:
        sys.exit("ERROR: CU_MULTI_ROOT is not set.")
        
    global_start: Decimal = min(start_time for (start_time, end_time) in robot_time_windows.values())
    global_end:   Decimal = max(e for (s, e) in robot_time_windows.values())

    id_int = 1
    for name in robot_time_windows.keys():
        env, robot_name = parse_env_and_robot(name)
        print(f"env: {env}, robot name: {robot_name}")

        lidar_in = os.path.join(DATA_ROOT, env, robot_name, f"{robot_name}_{env}_lidar")
        lidar_out = os.path.join(DATA_ROOT, env, robot_name, f"{robot_name}_{env}_lidar_trimmed")

        pose_in = os.path.join(DATA_ROOT, env, robot_name, f"{robot_name}_{env}_gt_rel_poses")
        pose_out = os.path.join(DATA_ROOT, env, robot_name, f"{robot_name}_{env}_gt_rel_poses_trimmed")

        start_time, end_time = robot_time_windows[name]
        print(f"start time: {start_time}, end time: {end_time}")

        # Clip poses (usually small; keep headers consistent too)
        clip_ros2_bag_dir(pose_in, 
                        pose_out, 
                        id_int,
                        start_time, 
                        end_time, 
                        global_start,
                        always_keep_topics=(),
                        verbose=True)
        id_int += 1

        # Clip lidar
        clip_ros2_bag_dir(lidar_in, 
                        lidar_out, 
                        id_int, 
                        start_time, 
                        end_time, 
                        global_start,
                        always_keep_topics=("ouster/metadata",),
                        verbose=True)
        id_int += 1