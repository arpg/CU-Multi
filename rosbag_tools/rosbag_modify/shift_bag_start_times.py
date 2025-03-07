"""
    multi_robot_bag.py
"""
import rosbag
import rospy
import os

def shift_bag_start_time(target_time_bag_path, source_time_bag_path, adjusted_time_bag_path):
    """
    Shifts the start time of a ROS bag file to a specific Unix timestamp.

    Parameters:
        input_bag_path (str): Path to the input bag file.
        output_bag_path (str): Path to the output bag file.
        new_start_time (float): The desired start time in Unix timestamp.
    """
    with rosbag.Bag(target_time_bag_path, 'r') as inbag:
        # Get the current start time of the bag
        new_start_time = inbag.get_start_time()

    with rosbag.Bag(source_time_bag_path, 'r') as inbag:
        # Get the current start time of the bag
        current_start_time = inbag.get_start_time()

        # Calculate the shift needed
        time_shift = new_start_time - current_start_time

    with rosbag.Bag(adjusted_time_bag_path, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(source_time_bag_path).read_messages():
            # print(f"time shift: {rospy.Duration(time_shift)}")

            # Shift the timestamp of the message
            new_time = t + rospy.Duration(time_shift)
            
            # Adjust the message header timestamps
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                msg.header.stamp += rospy.Duration(time_shift)

            # Adjust transform timestamps
            if hasattr(msg, 'transforms'):
                for transform in msg.transforms:
                    original_transform_stamp = transform.header.stamp
                    transform.header.stamp += rospy.Duration(time_shift)
            
            # Write the message with the new timestamp to the output bag
            outbag.write(topic, msg, new_time)

def shift_start_times(env, robot_prefixes, root_input_dir, root_output_dir):
    target_time_bag_path = f"{root_output_dir}/{robot_prefixes[0]}_{env}_filtered.bag"
    for robot_prefix in robot_prefixes[1:]:
        source_time_bag_path = f"{root_input_dir}/{robot_prefix}_{env}_filtered.bag"
        robot_output_bag_path = f"{root_output_dir}/{robot_prefix}_{env}_retimed.bag"
        print(f"\nShifting start time of {robot_prefix}...")
        shift_bag_start_time(target_time_bag_path, source_time_bag_path, robot_output_bag_path)
        print(f"Removing original bag for {robot_prefix}...")
        os.remove(source_time_bag_path)

def main():
    env = "main_campus"
    root_input_dir = f"/root/data/{env}/rosbags/filtered"
    root_output_dir = f"/root/data/{env}/rosbags/filtered"
    robot_prefixes = ['robot1', 'robot3', 'robot4'] #, 'robot3', 'robot4']

    shift_start_times(env, robot_prefixes, root_input_dir, root_output_dir)

if __name__ == '__main__':
    main()