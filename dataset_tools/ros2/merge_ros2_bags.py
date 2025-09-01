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

def merge_ros2_bags(input_dir: str,
                    output_dir: str,
                    storage_id: str = 'sqlite3'):

    # --- STEP 1: discover bag directories
    print("\n\n 1. GETTING BAGS.")
    bag_dirs = [os.path.join(input_dir, d)
                for d in os.listdir(input_dir)
                if os.path.isdir(os.path.join(input_dir, d))]
    if not bag_dirs:
        raise RuntimeError(f"No bag directories found in {input_dir}")

    # --- STEP 2: scan metadata to register topics
    print("\n\n 2. SCANNING.")
    topic_types = {}
    for bag in bag_dirs:
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
    for idx, bag in enumerate(bag_dirs):
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
    out_storage = StorageOptions(uri=output_dir, storage_id=storage_id)
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

    print(f"✅ Merged {count} messages into '{output_dir}' in chronological order.")

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <input_bags_dir> <output_bag_dir>")
        sys.exit(1)
    in_dir, out_dir = sys.argv[1], sys.argv[2]
    merge_ros2_bags(in_dir, out_dir)
