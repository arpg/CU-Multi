import rosbag
import rospy
from rosgraph_msgs.msg import Clock

topics_to_ignore = ["/ouster/range_image", 
                    "/ouster/reflec_image", 
                    "/ouster/nearir_image",
                    "/ouster/signal_image", 
                    "/ouster/scan",
                    "/lio_sam/mapping/cloud_registered",
                    "/lio_sam/mapping/cloud_registered_raw",
                    "/lio_sam/feature/cloud_surface",
                    "/lio_sam/feature/cloud_corner",
                    "/lio_sam/deskew/cloud_deskewed",
                    ]

def fix_tf_times(input_bag, output_bag):
    with rosbag.Bag(output_bag, 'w') as outbag:
        with rosbag.Bag(input_bag, 'r') as inbag:
            for topic, msg, t in inbag.read_messages():
                if topic in topics_to_ignore:
                    continue

                new_pub_time = None
                if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                    # print(f"topic: {topic}")
                    # print(f"    - bag timestamp: {t}")
                    # print(f"    - msg timestamp: {msg.header.stamp}")
                    new_pub_time = msg.header.stamp
                    # print(f"    - New bag timestamp: {new_pub_time}\n")

                # Fix tfs
                if hasattr(msg, 'transforms') and topic == "/tf":
                    for transform in msg.transforms:
                        # print(f"topic: {topic}")
                        # print(f"    - bag timestamp: {t}")
                        # print(f"    - tf timestamp: {transform.header.stamp}")
                        # print(f"    - number of tfs: {len(msg.transforms)}")
                        new_pub_time = transform.header.stamp
                        # print(f"    - New bag timestamp: {new_pub_time}\n")

                # Track the latest /clock time
                if topic == "/clock": # and isinstance(msg, Clock):
                    print(f"topic: {topic}")
                    print(f"    - bag timestamp: {t}")
                    print(f"    - clock timestamp: {msg.clock}\n")
                    new_pub_time = msg.clock

                if new_pub_time:
                    time_diff = abs(new_pub_time.to_sec() - t.to_sec())
                    if time_diff > 1.0:
                        # print(f"new_pub_time: {new_pub_time}")
                        # print(f"topic: {topic}")
                        # print(f"    - Time difference {time_diff} exceeds 1 second.\n")
                        outbag.write(topic, msg, t)
                    else:
                        outbag.write(topic, msg, new_pub_time)
                else:
                    # print(f"new_pub_time is None: {new_pub_time}")
                    # print(f"topic: {topic}")
                    outbag.write(topic, msg, t)

if __name__ == "__main__":
    input_bag = '/media/donceykong/edd/ec_courtyard_02_liosam_hm.bag'
    output_bag_static_removed = '/media/donceykong/edd/ec_courtyard_02_liosam_hm_fixed_tfs.bag'

    # Fix message times
    fix_tf_times(input_bag, output_bag_static_removed)
