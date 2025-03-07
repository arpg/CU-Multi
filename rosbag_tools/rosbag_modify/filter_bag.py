import rosbag

# Input and output file paths
input_bag_path = "/root/data/north_campus/rosbags/raw/north_campus_raw.bag"
output_bag_path = "/root/data/north_campus/rosbags/filtered/north_campus_filtered.bag"

# List of specific topics and namespaces to KEEP
namespaces = ['/gnss_1/', '/gnss_2/', '/imu/']  # List of namespaces
specific_topics = ['/ouster/metadata', '/ouster/points']  # List of specific topics

# Open the input bag and create the output bag
with rosbag.Bag(output_bag_path, 'w') as outbag:
    with rosbag.Bag(input_bag_path, 'r') as inbag:
        for topic, msg, t in inbag.read_messages():
            # Check if the topic matches any specific topics or belongs to specified namespaces
            if topic in specific_topics or any(topic.startswith(namespace) for namespace in namespaces):
                outbag.write(topic, msg, t)
