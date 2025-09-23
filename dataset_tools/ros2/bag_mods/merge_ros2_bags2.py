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

def merge_ros2_bags(ros2_bags, output_bag: str, storage_id: str = 'sqlite3'):
    """Function to merge a list of bags into a final bag for playback"""

    # --- STEP 1: scan metadata to register topics
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
        # reader.close()

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
                # idx,
                topic_name,
                msg_type,
                out_conv.output_serialization_format,
                # []
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

    # # --- CLEANUP
    # for rdr in readers:
    #     rdr.close()
    writer.close()

    print(f"✅ Merged {count} messages into '{output_bag}' in chronological order.")


if __name__ == '__main__':

    DATA_ROOT = os.getenv("CU_MULTI_ROOT")
    if DATA_ROOT is None:
        sys.exit(
            "ERROR: Environment variable CU_MULTI_ROOT is not set.\n"
            "Please set it before running the script, e.g.:\n"
            "  export CU_MULTI_ROOT=/your/custom/path\n"
        )
    print("CU_MULTI_ROOT is:", DATA_ROOT)

    env = "main_campus"
    robots = [1, 2]

    CREATE_ROBOT_BAG = False
    CREATE_ENV_BAG = True
    SENSORS = ["lidar", "gt_rel_poses"] # [lidar, poses, rgb, depth, gps]

    if CREATE_ENV_BAG:
        env_bags = []
        robots_string = "_".join(map(str, robots))
        env_bag_path = os.path.join(DATA_ROOT, f"{env}", f"{env}_robots{robots_string}_merged_bag")

        robots_info_string = ", ".join(map(str, robots))
        print(f"\nCreating {env} ENVIRONMENT bag with robots {robots_info_string}.")
        print(f"    - Path: {env_bag_path}")

    for robot in robots:
        robot_bags = []
        robot_dir = os.path.join(DATA_ROOT, f"{env}/robot{robot}")
        for sensor in SENSORS:
            sensor_bag = os.path.join(robot_dir, f"robot{robot}_{env}_{sensor}")
            robot_bags.append(sensor_bag)

        if CREATE_ROBOT_BAG:
            sensors_string = "_".join(map(str, SENSORS))
            robot_bag_path = os.path.join(robot_dir, f"robot{robot}_{env}_{sensors_string}")

            sensors_INFO_string = ", ".join(map(str, SENSORS))
            print(f"\nCreating {env} ROBOT{robot} bag with sensors {sensors_INFO_string}.")
            print(f"    - Path: {robot_bag_path}")
            merge_ros2_bags(env_bags, robot_bag_path)
            
        if CREATE_ENV_BAG:
            env_bags.extend(robot_bags)

    merge_ros2_bags(env_bags, env_bag_path)