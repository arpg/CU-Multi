"""
    multi_robot_bag.py
"""
import rosbag
import rospy
import os

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
                    # if transform.header.frame_id != "map":
                    transform.header.frame_id = add_prefix(transform.header.frame_id, prefix)
                    transform.child_frame_id = add_prefix(transform.child_frame_id, prefix)
            
            excluded_topics = [
                '/clock', '/tf', '/tf_static', '/rosout', '/rosout_agg',
                '/clicked_point', '/initialpose', '/move_base_simple/goal'
            ]
            if topic not in excluded_topics:
                topic = f"{prefix.rstrip('_')}{topic}"
                if hasattr(msg, 'header') and hasattr(msg.header, 'frame_id'): # and msg.header.frame_id != "map": 
                    new_msg_frame_id = add_prefix(msg.header.frame_id, prefix)
                    msg.header.frame_id = change_frame_id(msg.header.frame_id, new_msg_frame_id)
            outbag.write(topic, msg, t)

def main():
    root_input_dir = '/root/data/kittredge_loop/rosbags/liosam'
    root_output_dir = '/root/data/kittredge_loop/rosbags/liosam'
    robot_prefixes = ['robot1']

    for robot_prefix in robot_prefixes:
        robot_input_bag_path = f'{root_input_dir}/{robot_prefix}_kittredge_loop_filtered_liosam.bag'
        robot_output_bag_path = f'{root_output_dir}/{robot_prefix}_kittredge_loop_liosam.bag'

        print(f"\nCreating namespace and tf_prefix for {robot_prefix}...")
        add_tf_prefix_and_namespace(robot_input_bag_path, robot_output_bag_path, robot_prefix)
        if os.path.exists(robot_output_bag_path):
            os.remove(robot_input_bag_path)
        print("Done.")

if __name__ == '__main__':
    main()