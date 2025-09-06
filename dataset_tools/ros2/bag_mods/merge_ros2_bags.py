#!/usr/bin/env python3
import os
import sys
import heapq
import rosbag2_py
from rosbag2_py import StorageOptions, ConverterOptions, SequentialReader, SequentialWriter

def open_reader(bag_path):
    storage_opts = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_opts = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader = SequentialReader()
    reader.open(storage_opts, converter_opts)
    return reader, reader.get_all_topics_and_types()

def merge_bags(input_paths, output_path):
    # 1) Open readers
    readers = [open_reader(p) for p in input_paths]

    # 2) Prepare writer
    writer = SequentialWriter()
    out_storage = StorageOptions(uri=output_path, storage_id='sqlite3')
    conv_opts = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    writer.open(out_storage, conv_opts)

    # ← NEW: collect and register *all* topics from all bags
    topic_map = {}
    for _, topics_and_types in readers:               # each topics_and_types is a list of metadata
        for topic in topics_and_types:
            topic_map[topic.name] = topic             # overrides duplicates by name

    for topic_meta in topic_map.values():
        writer.create_topic(topic_meta)

    # 3) Seed the heap, merge, write…
    heap = []
    for reader, _ in readers:
        if reader.has_next():
            topic, data, timestamp = reader.read_next()
            heapq.heappush(heap, (timestamp, topic, data, reader))

    while heap:
        t, topic, data, reader = heapq.heappop(heap)
        writer.write(topic, data, t)
        if reader.has_next():
            topic2, data2, t2 = reader.read_next()
            heapq.heappush(heap, (t2, topic2, data2, reader))

if __name__ == '__main__':
    data_root = "/home/donceykong/Data/cu_multi"
    env = "main_campus"
    robots = [1, 2]

    # for robot in robots:
    #     bag_dir = os.path.join(data_root, f"{env}/robot{robot}")
    #     bag1 = os.path.join(bag_dir, f"robot{robot}_{env}_lidar")
    #     bag2 = os.path.join(bag_dir, f"robot{robot}_{env}_poses_CSV")
    #     merged_bag = os.path.join(bag_dir, f"robot{robot}_{env}_lidar_poses_CSV")

    #     in_bags = [bag1, bag2]
    #     merge_bags(in_bags, merged_bag)
    
    in_bags = []
    merged_bag = os.path.join(data_root, f"{env}", f"{env}_merged_bag_NEW")
    for robot in robots:
        bag_dir = os.path.join(data_root, f"{env}/robot{robot}")
        bag = os.path.join(bag_dir, f"robot{robot}_{env}_lidar_poses_CSV")
        in_bags.append(bag)
        # bag1 = os.path.join(bag_dir, f"robot{robot}_{env}_lidar")
        # bag2 = os.path.join(bag_dir, f"robot{robot}_{env}_poses")
        # in_bags.append(bag1)
        # in_bags.append(bag2)

    merge_bags(in_bags, merged_bag)