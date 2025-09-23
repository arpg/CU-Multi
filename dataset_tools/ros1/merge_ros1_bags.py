#!/usr/bin/env python3
import os
import sys
import heapq
import rosbag
import rospy

def merge_ros1_bags(input_dir: str, output_bag: str):
    # --- STEP 1: discover .bag files
    print("\n\n 1. GETTING BAGS.")
    bag_files = sorted([
        os.path.join(input_dir, f)
        for f in os.listdir(input_dir)
        if f.endswith('.bag') and os.path.isfile(os.path.join(input_dir, f))
    ])
    if not bag_files:
        raise RuntimeError(f"No .bag files found in {input_dir}")

    # --- STEP 2: open each bag and prime the first message into a heap
    print("\n\n 2. PRIMING.")
    readers = []   # [(rosbag.Bag, iterator), ...]
    heap = []      # will hold tuples (timestamp_ns, reader_idx, topic, msg)
    for idx, bag_path in enumerate(bag_files):
        bag = rosbag.Bag(bag_path, 'r')
        it  = bag.read_messages()
        try:
            topic, msg, t = next(it)
            ts_ns = t.secs * 1_000_000_000 + t.nsecs
            heapq.heappush(heap, (ts_ns, idx, topic, msg))
            readers.append((bag, it))
        except StopIteration:
            # this bag was empty
            bag.close()

    # --- STEP 3: open the output bag for writing
    print("\n\n 3. WRITING.")
    out_bag = rosbag.Bag(output_bag, 'w')

    # --- STEP 4: k-way merge via a min-heap
    print("\n\n 4. K-WAY MERGING.")
    count = 0
    while heap:
        ts_ns, idx, topic, msg = heapq.heappop(heap)
        # reconstruct rospy.Time from nanoseconds
        secs  = ts_ns // 1_000_000_000
        nsecs = ts_ns % 1_000_000_000
        t = rospy.Time(int(secs), int(nsecs))

        # write the message into the merged bag
        out_bag.write(topic, msg, t)
        count += 1

        # pull the next message from the same reader
        bag, it = readers[idx]
        try:
            topic, msg, t = next(it)
            ts_ns = t.secs * 1_000_000_000 + t.nsecs
            heapq.heappush(heap, (ts_ns, idx, topic, msg))
        except StopIteration:
            # that bag is exhausted
            bag.close()

    # --- STEP 5: cleanup
    out_bag.close()
    print(f"\n✅ Merged {count} messages into '{output_bag}' in chronological order.")

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <input_bags_dir> <output_bag_file>")
        sys.exit(1)
    merge_ros1_bags(sys.argv[1], sys.argv[2])

