"""
    multi_robot_bag.py
"""
import rosbag
import rospy
import os

def merge_bags_og(bag1_path, bag2_path, output_bag_path):
    """
    Merges two ROS bag files into a single bag file.

    Parameters:
        bag1_path (str): Path to the first input bag file.
        bag2_path (str): Path to the second input bag file.
        output_bag_path (str): Path to the output bag file.
    """
    with rosbag.Bag(output_bag_path, 'w') as outbag:
        for bag_path in [bag1_path, bag2_path]:
            with rosbag.Bag(bag_path, 'r') as inbag:
                for topic, msg, t in inbag.read_messages():
                    outbag.write(topic, msg, t)

def merge_bags(bag_paths, output_bag_path):
    """
    Merges multiple ROS bag files into a single bag file.

    Parameters:
        bag_paths (list of str): List of paths to the input bag files.
        output_bag_path (str): Path to the output bag file.
    """
    with rosbag.Bag(output_bag_path, 'w') as outbag:
        for bag_path in bag_paths:
            with rosbag.Bag(bag_path, 'r') as inbag:
                for topic, msg, t in inbag.read_messages():
                    outbag.write(topic, msg, t)

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
    
def add_tf_prefix_and_namespace(input_bag_path, output_bag_path, prefix):
    """
    Adds a tf_prefix to all tf messages in a ROS bag file.

    Parameters:
        input_bag_path (str): Path to the input bag file.
        output_bag_path (str): Path to the output bag file.
        prefix (str): Prefix to add to each frame_id and child_frame_id in tf messages.
    """
    def add_prefix(frame_id, prefix):
        if frame_id.startswith('/'):
            return f"/{prefix}_{frame_id}"
        else:
            return f"{prefix}_{frame_id}"
        
    def change_frame_id(frame_id, new_frame_id):
        return new_frame_id if frame_id else frame_id
    
    with rosbag.Bag(output_bag_path, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(input_bag_path).read_messages():
            # print(f"\ntopic: {topic}")
            if topic == "/tf" or topic == "/tf_static":
                for transform in msg.transforms:
                    if transform.header.frame_id != "map":
                        transform.header.frame_id = add_prefix(transform.header.frame_id, prefix)
                    transform.child_frame_id = add_prefix(transform.child_frame_id, prefix)
            
            excluded_topics = [
                '/clock', '/tf', '/tf_static', '/rosout', '/rosout_agg',
                '/clicked_point', '/initialpose', '/move_base_simple/goal'
            ]
            if topic not in excluded_topics:
                topic = f"{prefix.rstrip('_')}{topic}"
                if hasattr(msg, 'header') and hasattr(msg.header, 'frame_id') and msg.header.frame_id != "map": 
                    new_msg_frame_id = add_prefix(msg.header.frame_id, prefix)
                    msg.header.frame_id = change_frame_id(msg.header.frame_id, new_msg_frame_id)
            outbag.write(topic, msg, t)

def create_multirobot_bag(robot_prefixes, root_input_dir, root_output_dir):
    for robot_prefix in robot_prefixes:
        robot_input_bag_path = f'{root_input_dir}/{robot_prefix}_kittredge_filtered_liosam.bag'
        robot_output_bag_path = f'{root_output_dir}/{robot_prefix}_cu_main.bag'

        print(f"\nCreating namespace and tf_prefix for {robot_prefix}...")
        # add_tf_prefix_and_namespace(robot_input_bag_path, robot_output_bag_path, robot_prefix)
        print("Done.")

    # # Needs adjusting for 3 or more robots
    target_time_bag_path = f'{root_output_dir}/{robot_prefixes[3]}_cu_main.bag'
    bags_to_merge = [target_time_bag_path]
    for robot_prefix in robot_prefixes[:3]:
        # source_time_bag_path = f'{root_output_dir}/{robot_prefix}_cu_main.bag'
        adjusted_time_bag_path = f'{root_output_dir}/{robot_prefix}_cu_main_time_shifted.bag'
        bags_to_merge.append(adjusted_time_bag_path)
        print(f"\nShifting start time of {robot_prefix}...")
        # shift_bag_start_time(target_time_bag_path, source_time_bag_path, adjusted_time_bag_path)
        # os.remove(source_time_bag_path)
        print("Done.")

    print(f"\nMerging all bags ...")
    merged_bag_path = f'{root_output_dir}/robots_all_cu_main.bag'
    merge_bags(bags_to_merge, merged_bag_path)
    print(f"Done. Final bag saved to {merged_bag_path}")

    # print(f"\nRemoving remaining unused bags ...")
    # for bag in bags_to_merge:
    #     os.remove(bag)
    # print(f"Done.")

def main():
    root_input_dir = '/media/donceykong/edd'
    root_output_dir = '/media/donceykong/puck_of_destiny'
    robot_prefixes = ['robot1', 'robot2', 'robot3', 'robot4']

    create_multirobot_bag(robot_prefixes, root_input_dir, root_output_dir)

if __name__ == '__main__':
    main()