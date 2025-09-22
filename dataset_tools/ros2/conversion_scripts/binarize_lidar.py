#!/usr/bin/env python3
import os
import sys
import numpy as np
from decimal import Decimal
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2


def pointcloud_msg_to_numpy(msg, datatype=np.float32):
    fields = ['x', 'y', 'z', 'intensity']
    points = pc2.read_points(msg, field_names=fields, skip_nans=False)

    # Convert generator to plain ndarray (N, 4)
    pointcloud_numpy = np.array([ [p[0], p[1], p[2], p[3]] for p in points ], dtype=datatype)

    return pointcloud_numpy


def save_ouster_pointcloud(lidar_pc_bin_path, pc2_ros2_msg, lidar_pc_number):
    pointcloud = pointcloud_msg_to_numpy(pc2_ros2_msg)
    filename = f"{lidar_pc_bin_path}/unsorted_lidar_pointcloud_{lidar_pc_number:010d}.bin"
    pointcloud.tofile(filename)


def read_robot_bag(env, robot, robot_dir):
    lidar_ts_index_dict = {}
    lidar_pc_number = 0

    lidar_bag = os.path.join(robot_dir, f"robot{robot}_{env}_lidar")

    # Make dirs
    lidar_bin_dir = os.path.join(robot_dir, "lidar_bin")
    lidar_data_dir = os.path.join(lidar_bin_dir, "data")
    os.makedirs(lidar_data_dir, exist_ok=True)
    lidar_ts_path = os.path.join(lidar_bin_dir, "timestamps.txt")

    # Open the bag
    reader = SequentialReader()
    storage_options = StorageOptions(uri=lidar_bag, storage_id="")
    converter_options = ConverterOptions("cdr", "cdr")
    reader.open(storage_options, converter_options)

    # Get topic type
    topic_name = f"robot{robot}/ouster/points"
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_type = get_message(type_map[topic_name])

    while reader.has_next():
        topic, data, bag_ns = reader.read_next()
        if topic != topic_name:
            continue

        msg = deserialize_message(data, msg_type)
        sec = msg.header.stamp.sec
        nsec = msg.header.stamp.nanosec
        header_sec = Decimal(sec) + Decimal(nsec) / Decimal(1e9)

        lidar_ts_index_dict[header_sec] = lidar_pc_number
        save_ouster_pointcloud(lidar_data_dir, msg, lidar_pc_number)
        lidar_pc_number += 1

    # Save sorted timestamps
    timestamps_np = np.array(sorted(lidar_ts_index_dict.keys()))
    np.savetxt(lidar_ts_path, timestamps_np, fmt='%s')

    # Rename files in order
    for idx, timestamp in enumerate(timestamps_np):
        orig_index = lidar_ts_index_dict[timestamp]
        orig_file = f"{lidar_data_dir}/unsorted_lidar_pointcloud_{orig_index:010d}.bin"
        new_file = f"{lidar_data_dir}/{idx:010d}.bin"
        os.rename(orig_file, new_file)
        if orig_index != idx:
            print(f"    - Orig index: {orig_index}, new index: {idx}")

    print(f" - Total messages on {topic_name}: {lidar_pc_number}")


if __name__ == '__main__':
    DATA_ROOT = os.getenv("CU_MULTI_ROOT")
    if DATA_ROOT is None:
        sys.exit("ERROR: CU_MULTI_ROOT not set")

    env = "main_campus"
    robots = [4]

    for robot in robots:
        print(f"\nBinarizing lidar data for robot{robot}.")
        robot_dir = os.path.join(DATA_ROOT, f"{env}/robot{robot}")
        read_robot_bag(env, robot, robot_dir)
        print(" - Complete")
