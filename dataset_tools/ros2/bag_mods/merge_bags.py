#!/usr/bin/env python3
"""
Merge multiple ROS 2 bag directories into a single output bag.

Tested with ROS 2 Humble / Iron APIs (rosbag2_py). It performs a simple
concatenation: it writes all messages from bag1, then bag2, etc., preserving
per-message timestamps. Topic schemas (name/type/serialization format) are
created on first sight; subsequent bags must match the same type or will be
skipped (unless --allow-type-mismatch is set to force-create a new renamed
topic).

Usage:
  python3 merge_ros2_bags.py -o /path/to/output_bag /path/to/bag1 /path/to/bag2 [...]

Notes:
- The output path is a bag directory (it will be created). Do NOT point to
  a file; rosbag2 will manage files within the directory.
- This script does not globally sort by timestamp across inputs (that can be
  expensive). The resulting bag will contain original timestamps; many tools
  rely on timestamps rather than file order. If you need global time ordering,
  record with synchronized clocks or post-process with a dedicated sorter.
- Compression: you can choose the storage plugin and compression mode via
  flags; defaults match common sqlite3/CDR setups.
"""
import argparse
import os
import shutil
import sys
from typing import Dict, Set

try:
    import rosbag2_py
except Exception as e:
    sys.stderr.write(
        "\n[ERROR] Could not import rosbag2_py. Make sure you're running inside a ROS 2 environment (e.g., `source /opt/ros/<distro>/setup.bash`).\nOriginal error: %s\n" % e
    )
    sys.exit(1)


def open_reader(bag_path: str) -> rosbag2_py.SequentialReader:
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id="sqlite3",
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def open_writer(out_path: str, storage_id: str, compression_mode: str, compression_format: str) -> rosbag2_py.SequentialWriter:
    storage_options = rosbag2_py.StorageOptions(
        uri=out_path,
        storage_id=storage_id,
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    writer = rosbag2_py.SequentialWriter()

    # Configure compression if requested and supported
    if hasattr(rosbag2_py, "CompressionOptions"):
        comp = rosbag2_py.CompressionOptions(
            compression_mode=compression_mode,
            compression_format=compression_format,
            compression_queue_size=1,
            compression_threads=0,
        )
        try:
            writer.open(storage_options, converter_options, comp)
            return writer
        except TypeError:
            # Older distros don't support passing CompressionOptions to open()
            pass

    writer.open(storage_options, converter_options)
    return writer


def normalize_topic_name(name: str) -> str:
    # Ensure topic names always start with '/'
    return name if name.startswith('/') else '/' + name


def merge_bags(output: str, inputs: list, allow_type_mismatch: bool, storage_id: str, compression_mode: str, compression_format: str, prefix_each_bag: bool) -> None:
    # if os.path.exists(output):
    #     # rosbag2 expects to create the directory. If it exists and has files, bail.
    #     if os.listdir(output):
    #         raise RuntimeError(f"Output directory '{output}' already exists and is not empty. Use a new path.")
    #     # If empty dir, we'll reuse it.
    # else:
    #     os.makedirs(output, exist_ok=True)

    writer = open_writer(output, storage_id, compression_mode, compression_format)

    created_topics: Dict[str, rosbag2_py.TopicMetadata] = {}
    all_types_for_topic: Dict[str, Set[str]] = {}

    for bag_idx, bag in enumerate(inputs):
        if not os.path.isdir(bag):
            raise FileNotFoundError(f"Input bag directory not found: {bag}")
        print(f"[INFO] Reading bag {bag_idx+1}/{len(inputs)}: {bag}")

        reader = open_reader(bag)
        topics = reader.get_all_topics_and_types()

        # Create topics in writer if needed
        for t in topics:
            orig_name = normalize_topic_name(t.name)
            topic_name = orig_name
            if prefix_each_bag:
                topic_name = f"/bag{bag_idx+1}{orig_name}"

            tm = rosbag2_py.TopicMetadata(
                name=topic_name,
                type=t.type,
                serialization_format=t.serialization_format or "cdr",
            )

            # Track types seen per (target) topic
            types = all_types_for_topic.setdefault(topic_name, set())
            types.add(t.type)

            if topic_name in created_topics:
                # Check for type mismatch
                if t.type != created_topics[topic_name].type:
                    msg = (
                        f"[WARN] Type mismatch for topic {topic_name}: "
                        f"existing {created_topics[topic_name].type} vs incoming {t.type}"
                    )
                    if allow_type_mismatch:
                        # Create a unique renamed topic for this mismatch
                        alt_name = f"{topic_name}__bag{bag_idx+1}_type_{t.type.replace('/', '_')}"
                        print(msg + f" -> writing incoming messages to '{alt_name}'")
                        alt_tm = rosbag2_py.TopicMetadata(
                            name=alt_name,
                            type=t.type,
                            serialization_format=t.serialization_format or "cdr",
                        )
                        if alt_name not in created_topics:
                            writer.create_topic(alt_tm)
                            created_topics[alt_name] = alt_tm
                        # Store a mapping so we can remap messages on write
                        t._target_name = alt_name  # attach attribute for later
                    else:
                        print(msg + " -> skipping this topic in this bag")
                        t._skip = True
                else:
                    # ok
                    pass
            else:
                writer.create_topic(tm)
                created_topics[topic_name] = tm
                t._target_name = topic_name

        # Stream messages
        msgs_written = 0
        while reader.has_next():
            m = reader.read_next()
            # m has fields: topic_name, serialized_data (rclpy.SerializedMessage), time_stamp
            # Adjust topic if we prefixed or had to rename due to type mismatch
            target_topic = normalize_topic_name(m.topic_name)
            if prefix_each_bag:
                target_topic = f"/bag{bag_idx+1}{target_topic}"

            # If this topic marked to skip due to type mismatch
            # Build a quick lookup per bag read for efficiency
            # We consult topics list to see if a remap or skip is needed
            remapped = False
            for t in topics:
                tn = normalize_topic_name(t.name)
                tt = tn
                if prefix_each_bag:
                    tt = f"/bag{bag_idx+1}{tn}"
                if tt == target_topic:
                    if getattr(t, "_skip", False):
                        remapped = True  # signal skip
                        target_topic = None
                    elif hasattr(t, "_target_name") and t._target_name != tt:
                        remapped = True
                        target_topic = t._target_name
                    break

            if target_topic is None:
                # skip write
                continue

            # Create a new SerializedBagMessage to safely override topic_name if needed
            sbm = rosbag2_py.SerializedBagMessage()
            sbm.topic_name = target_topic
            sbm.time_stamp = m.time_stamp
            sbm.serialized_data = m.serialized_data

            writer.write(sbm)
            msgs_written += 1

        print(f"[INFO] Wrote {msgs_written} messages from {bag}")

    print(f"[DONE] Merged {len(inputs)} bag(s) into: {output}")


def main():
    parser = argparse.ArgumentParser(description="Merge multiple ROS 2 bags into one output bag")
    parser.add_argument("inputs", nargs='+', help="Input rosbag2 directories to merge (order matters)")
    parser.add_argument("-o", "--output", required=True, help="Output rosbag2 directory to create")
    parser.add_argument("--storage", default="sqlite3", help="rosbag2 storage plugin (default: sqlite3)")
    parser.add_argument("--compression-mode", default="none", choices=["none", "file", "message"], help="Compression mode for output (if supported)")
    parser.add_argument("--compression-format", default="zstd", choices=["zstd", "lz4"], help="Compression format (if supported)")
    parser.add_argument("--allow-type-mismatch", action="store_true", help="If a topic type differs across inputs, auto-rename the new stream instead of skipping")
    parser.add_argument("--prefix-each-bag", action="store_true", help="Prefix topics from each bag with /bagN to avoid collisions")

    args = parser.parse_args()

    try:
        merge_bags(
            output=args.output,
            inputs=args.inputs,
            allow_type_mismatch=args.allow_type_mismatch,
            storage_id=args.storage,
            compression_mode=args.compression_mode,
            compression_format=args.compression_format,
            prefix_each_bag=args.prefix_each_bag,
        )
    except Exception as e:
        sys.stderr.write(f"[ERROR] {e}\n")
        sys.exit(2)


if __name__ == "__main__":
    main()
