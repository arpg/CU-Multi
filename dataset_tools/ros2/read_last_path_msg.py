import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from nav_msgs.msg import Path
import sys
import os
import csv
from decimal import Decimal, getcontext, ROUND_HALF_EVEN


getcontext().prec = 40
NSEC = Decimal("1000000000")
EPOCH_THRESH = Decimal("1000000000")  # ~2001+


def get_last_message(bag_path, topic_name):
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path, 
        storage_id="sqlite3"
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Map topic types
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    if topic_name not in type_map:
        print(f"Topic {topic_name} not found in bag.")
        return None

    msg_type = get_message(type_map[topic_name])
    last_msg = None
    last_stamp = None

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == topic_name:
            msg = deserialize_message(data, msg_type)
            last_msg = msg
            last_stamp = t

    return last_stamp, last_msg


def rostime_to_sec(sec: int, nanosec: int) -> Decimal:
    """Convert ROS 2 builtin_interfaces/Time into Decimal seconds."""
    return Decimal(sec) + (Decimal(nanosec) / NSEC)


def ensure_csv_with_header(csv_path: str):
    exists = os.path.exists(csv_path)
    if not exists or os.path.getsize(csv_path) == 0:
        # create and write header
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp_sec", "x", "y", "z", "qx", "qy", "qz", "qw"])


def fmt(v, places=10, strip=False):
    s = f"{float(v):.{places}f}"
    return s.rstrip('0').rstrip('.') if strip else s


def add_pose_to_csv(csv_file, pose_msg):
    """
    pose: geometry_msgs.msg.PoseStamped(
        header=std_msgs.msg.Header(
            stamp=builtin_interfaces.msg.Time(sec=1722895176, nanosec=888656139), 
            frame_id='robot1_odom'
        ),
        pose=geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x, y, z), 
            orientation=geometry_msgs.msg.Quaternion(x, y, z, w)
        )
    )

    CSV out will have:
        timestamp (in sec), x, y, z, qx, qy, qz, qw
    """
    t_sec = rostime_to_sec(pose_msg.header.stamp.sec, pose_msg.header.stamp.nanosec)

    trans = pose_msg.pose.position
    x, y, z = fmt(trans.x), fmt(trans.y), fmt(trans.z)

    quat = pose_msg.pose.orientation
    qx, qy, qz, qw = fmt(quat.x), fmt(quat.y), fmt(quat.z), fmt(quat.w)

    # print(f"timestamp: {t_sec}")
    # print(f"x: {x}, y: {y}, z: {z}, qx: {qx}, qy: {qy}, qz: {qz}, qw: {qw}")

    with open(csv_file, "a", newline="") as f:
        w = csv.writer(f)
        # Write as strings to preserve Decimal precision in the CSV
        w.writerow([str(t_sec), x, y, z, qx, qy, qz, qw])

if __name__ == "__main__":
    data_root = "/home/donceykong/Data/cu_multi"
    env = "main_campus"
    robot_nums = [1]

    for robot_num in robot_nums:
        bag_dir = os.path.join(data_root, f"{env}/robot{robot_num}")
        poses_bag_path = os.path.join(bag_dir, f"robot{robot_num}_{env}_poses")
        lidar_bag_path = os.path.join(bag_dir, f"robot{robot_num}_{env}_lidar")
        topic_name = f"robot{robot_num}/lio_sam/mapping/path"

        # Get all lidar timestamps
        # HERE
        lidar_timestamps = 

        # CSV output per robot
        csv_file = os.path.join(
            bag_dir, f"robot{robot_num}_{env}_last_path_poses.csv"
        )
        ensure_csv_with_header(csv_file)

        stamp, msg = get_last_message(poses_bag_path, topic_name)
        if msg is not None:
            # print(f"Last message timestamp: {stamp}. len(poses): {len(msg.poses)}")
            for pose_msg in msg.poses:
                add_pose_to_csv(csv_file, pose_msg)
        else:
            print("No message found.")