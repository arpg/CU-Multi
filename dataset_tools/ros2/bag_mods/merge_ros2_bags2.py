#!/usr/bin/env python3
import os
import sys
import heapq
from rosbag2_py import (
    SequentialReader,
    SequentialWriter,
    StorageOptions,
    ConverterOptions,
    TopicMetadata
)

def merge_ros2_bags(ros2_bags,
                    output_bag: str,
                    storage_id: str = 'sqlite3'):

    # --- STEP 2: scan metadata to register topics
    print("\n\n 2. SCANNING.")
    topic_types = {}
    for bag in ros2_bags:
        reader = SequentialReader()
        reader.open(
            StorageOptions(uri=bag, storage_id=storage_id),
            ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )
        )
        for info in reader.get_all_topics_and_types():
            topic_types[info.name] = info.type
        reader.close()

    # --- STEP 3: open all readers and prime first messages
    print("\n\n 3. PRIMING.")
    readers = []
    heap = []  # will contain tuples (timestamp, reader_idx, topic, data)
    for idx, bag in enumerate(ros2_bags):
        rdr = SequentialReader()
        rdr.open(
            StorageOptions(uri=bag, storage_id=storage_id),
            ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )
        )
        if rdr.has_next():
            topic, data, ts = rdr.read_next()
            heapq.heappush(heap, (ts, idx, topic, data))
        readers.append(rdr)

    # --- STEP 4: open writer and register topics
    print("\n\n 4. WRITING.")
    writer = SequentialWriter()
    # define storage + converter options for the writer
    out_storage = StorageOptions(uri=output_bag, storage_id=storage_id)
    out_conv    = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    writer.open(out_storage, out_conv)
    for idx, (topic_name, msg_type) in enumerate(topic_types.items()):
        writer.create_topic(
            TopicMetadata(
                idx,
                topic_name,
                msg_type,
                out_conv.output_serialization_format,
                []
            )
        )


    # --- STEP 5: k-way merge via heap
    print("\n\n 5. K-WAY MERGING.")
    count = 0
    while heap:
        ts, idx, topic, data = heapq.heappop(heap)
        writer.write(topic, data, ts)
        count += 1

        rdr = readers[idx]
        if rdr.has_next():
            topic, data, ts = rdr.read_next()
            heapq.heappush(heap, (ts, idx, topic, data))

    # --- CLEANUP
    for rdr in readers:
        rdr.close()
    writer.close()

    print(f"✅ Merged {count} messages into '{output_bag}' in chronological order.")

if __name__ == '__main__':
    data_root = "/home/donceykong/Data/cu_multi"
    env = "main_campus"
    robots = [1, 2]

    in_bags = []
    merged_bag = os.path.join(data_root, f"{env}", f"{env}_merged_bag_NEW")
    for robot in robots:
        bag_dir = os.path.join(data_root, f"{env}/robot{robot}")
        bag1 = os.path.join(bag_dir, f"robot{robot}_{env}_poses_CSV")
        bag2 = os.path.join(bag_dir, f"robot{robot}_{env}_lidar")
        in_bags.append(bag1)
        in_bags.append(bag2)

    merge_ros2_bags(in_bags, merged_bag)