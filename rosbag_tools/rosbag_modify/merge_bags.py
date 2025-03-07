import rosbag
from datetime import datetime
import os

def merge_rosbags(input_bags, output_bag):
    # Open the output bag
    with rosbag.Bag(output_bag, 'w') as output_bag_handle:
        for input_bag in input_bags:
            with rosbag.Bag(input_bag, 'r') as bag:
                for topic, msg, t in bag.read_messages():
                    output_bag_handle.write(topic, msg, t)

    print(f"Bags {', '.join(input_bags)} merged into {output_bag}")

if __name__ == "__main__":
    robot = "robot4"
    input_dir = f"/root/data/rosbags/main_campus/raw/{robot}"
    merged_bag = f"/root/data/rosbags/main_campus/raw/{robot}_main_campus_raw.bag"
    input_bags = [os.path.join(input_dir, f) for f in os.listdir(input_dir)]

    merge_rosbags(input_bags, merged_bag)
