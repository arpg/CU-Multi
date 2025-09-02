#!/usr/bin/env python3
"""
Read a ROS 2 bag and, for every message from the topic "ouster/points",
print the message header timestamp and the bag-recorded timestamp.

Usage:
  python3 print_ouster_timestamps.py /path/to/bag
  # optional:
  python3 print_ouster_timestamps.py /path/to/bag --topic ouster/points
"""

import os
import argparse
import csv
import numpy as np

from decimal import Decimal
from collections import namedtuple
from datetime import datetime, timezone

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def ns_to_iso(ns: int) -> str:
    """Convert nanoseconds since Unix epoch to ISO 8601 (UTC)."""
    try:
        return datetime.fromtimestamp(ns / 1e9, tz=timezone.utc).isoformat()
    except Exception:
        return "n/a"


# for interpolation helper
PoseRecord = namedtuple("PoseRecord", ["time", "position", "orientation"])
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
    quaternions = np.stack([r.orientation for r in records])

    return times, positions, quaternions


def save_csv_poses(out_path, times, positions, quaternions):
    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        # optional header
        writer.writerow(["# time_sec", "x", "y", "z", "qx", "qy", "qz", "qw"])
        for t, pos, quat in zip(times, positions, quaternions):
            writer.writerow([
                f"{Decimal(t):.9f}",  # keep high precision seconds
                pos[0], pos[1], pos[2],
                quat[0], quat[1], quat[2], quat[3]
            ])


def fix_csv(bag_path, in_csv, out_csv):
    # Open the bag
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id="")  # empty = auto-detect (sqlite3/mcap)
    converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)

    # Map topic -> type
    topic_name = f"{args.robot_name}/{args.topic}" #if args.topic.startswith("/") else f"{args.topic}"
    topics_and_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics_and_types}

    if topic_name not in type_map:
        print(f'ERROR: Topic "{topic_name}" not found in bag.\nAvailable topics:')
        for t in topics_and_types:
            print(f"  {t.name}  [{t.type}]")
        return

    msg_type_str = type_map[topic_name]
    msg_type = get_message(msg_type_str)

    count = 0
    lidar_timestamps = []
    while reader.has_next():
        topic, data, bag_ns = reader.read_next()  # bag_ns is nanoseconds since epoch
        if topic != topic_name:
            continue

        msg = deserialize_message(data, msg_type)

        # Try to read header.stamp (sec, nanosec)
        header_ns = None
        try:
            sec = int(msg.header.stamp.sec)
            nsec = int(msg.header.stamp.nanosec)  # builtin_interfaces/Time uses "nanosec"
            header_sec = Decimal(sec) + Decimal(nsec) / Decimal(1_000_000_000)
            header_ns = sec * 1_000_000_000 + nsec
        except Exception:
            pass

        print(f"\nCount: {count}, Timestamp: {header_sec}")

        lidar_timestamps.append(header_sec)
        count += 1
    print(f"Total messages on {topic_name}: {count}")

    # Now, fix all the pose timestamps in below csv so that they are same as those in lidar_timestamps
    times, positions, quaternions, = load_csv_poses(in_csv)

    print(f"len(times): {len(times)}")
    times = lidar_timestamps
    print(f"New len(times): {len(times)}")

    # # Replace timestamps with lidar timestamps (index by index)
    # for i in range(min(len(times), len(lidar_timestamps))):
    #     times[i] = lidar_timestamps[i]

    save_csv_poses(out_csv, times, positions, quaternions)
    print(f"Saved aligned poses to {out_csv}")


def main():
    parser = argparse.ArgumentParser(description="Print timestamps for messages on a topic from a ROS 2 bag.")
    parser.add_argument("--env", default="main_campus", 
                        help="Environment from CU-Multi Dataset.")
    parser.add_argument("--robot_name", default="robot2", 
                        help="Robot name. Such as 'robot1', 'robot2', 'robot3', or 'robot4'.")
    parser.add_argument("--topic", default="ouster/points",
                        help='Topic name (with or without leading "/"). Default: ouster/points')
    args = parser.parse_args()

    # Root Data directory and CSV to fix
    root_path = "/media/donceykong/doncey_ssd_03/CU_MULTI"
    robot_dir = os.path.join(root_path, args.env, args.robot_name)
    in_csv = os.path.join(robot_dir, 
                          f"main_campus_{args.robot_name}_ref_NEW.csv")
    out_csv = os.path.join(robot_dir, 
                           f"main_campus_{args.robot_name}_ref_NEW_aligned.csv")
    bag_path = os.path.join(robot_dir, 
                            f"{args.robot_name}_{args.env}_lidar", 
                            f"{args.robot_name}_{args.env}_lidar_0.db3")


if __name__ == "__main__":
    main()

