import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import numpy as np
import struct
from tqdm import tqdm
import yaml
import os
from rclpy.time import Time
from collections import namedtuple

def labels2RGB(label_ids, labels_dict):
    """
        Get the color values for a set of semantic labels using their label IDs and the labels dictionary.

        Args:
            label_ids (np array of int): The semantic labels.
            labels_dict (dict): The dictionary containing the semantic label IDs and their corresponding RGB values.

        Returns:
            rgb_array (np array of float): The RGB values corresponding to the semantic labels.
    """
    # Prepare the output array
    # rgb_array = np.zeros((label_ids.shape[0], 3), dtype=float)
    rgb_array = np.zeros((len(label_ids), 3), dtype=float)
    for idx, label_id in enumerate(label_ids):
        if label_id in labels_dict:
            color = labels_dict.get(label_id, (0, 0, 0))  # Default color is black
            rgb_array[idx] = np.array(color)
    return rgb_array

Label = namedtuple(
    "Label",
    [
        "name",  # The identifier of this label, e.g. 'car', 'person', ... .
        "id",  # An integer ID that is associated with this label.
        "color",  # The color of this label
    ],
)


labels = [
    #       name, id, color
    Label("unlabeled", 0, (0, 0, 0)),               # OVERLAP
    Label("outlier", 1, (0, 0, 0)),
    Label("car", 10, (0, 0, 142)),                  # OVERLAP
    Label("bicycle", 11, (119, 11, 32)),            # OVERLAP
    Label("bus", 13, (250, 80, 100)),
    Label("motorcycle", 15, (0, 0, 230)),           # OVERLAP
    Label("on-rails", 16, (255, 0, 0)),
    Label("truck", 18, (0, 0, 70)),                 # OVERLAP
    Label("other-vehicle", 20, (51, 0, 51)),        # OVERLAP
    Label("person", 30, (220, 20, 60)),             # OVERLAP
    Label("bicyclist", 31, (200, 40, 255)),
    Label("motorcyclist", 32, (90, 30, 150)),
    Label("road", 40, (128, 64, 128)),              # OVERLAP
    Label("parking", 44, (250, 170, 160)),          # OVERLAP
    # Label("OSM BUILDING", 45, (0, 0, 255)),         # ************ OSM
    # Label("OSM ROAD", 46, (255, 0, 0)),             # ************ OSM
    Label("sidewalk", 48, (244, 35, 232)),          # OVERLAP
    Label("other-ground", 49, (81, 0, 81)),         # OVERLAP
    Label("building", 50, (0, 100, 0)),             # OVERLAP
    Label("fence", 51, (190, 153, 153)),            # OVERLAP
    Label("other-structure", 52, (0, 150, 255)),
    Label("lane-marking", 60, (170, 255, 150)),
    Label("vegetation", 70, (107, 142, 35)),        # OVERLAP
    Label("trunk", 71, (0, 60, 135)),
    Label("terrain", 72, (152, 251, 152)),          # OVERLAP
    Label("pole", 80, (153, 153, 153)),             # OVERLAP
    Label("traffic-sign", 81, (0, 0, 255)),
    Label("other-object", 99, (255, 255, 50)),
    # Label("moving-car", 252, (245, 150, 100)),
    # Label("moving-bicyclist", 253, ()),
    # Label("moving-person", 254, (30, 30, 25)),
    # Label("moving-motorcyclist", 255, (90, 30, 150)),
    # Label("moving-on-rails", 256, ()),
    # Label("moving-bus", 257, ()),
    # Label("moving-truck", 258, ()),
    # Label("moving-other-vehicle", 259, ()),
]


def rgb_to_float(r, g, b):
    """Packs RGB values into a float32 using PCL-style encoding."""
    rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)

    return struct.unpack('f', struct.pack('I', rgb_int))[0]


def get_semantics(msg, root_label_dir):
    """Returns np array of semantic indices for an input pc"""
    tgt_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    # Load all timestamps
    ts_file = os.path.join(root_label_dir, "timestamps.txt")
    with open(ts_file, "r") as f:
        timestamps = [float(line.strip()) for line in f]

    # Find matching index
    try:
        tgt_label_index = timestamps.index(tgt_timestamp)
    except ValueError:
        raise RuntimeError(f"Timestamp {tgt_timestamp:.20f} not found in {ts_file}")

    # Get label file path (e.g., labels/000123.bin)
    label_filename = f"lidar_pointcloud_{tgt_label_index + 1}.bin"
    label_filepath = os.path.join(root_label_dir, "labels", label_filename)

    if not os.path.exists(label_filepath):
        raise FileNotFoundError(f"Label file not found: {label_filepath}")

    # Read labels from .bin file
    labels_np = np.fromfile(label_filepath, dtype=np.int32).reshape(-1)

    return labels_np


def add_rgb_field_to_pointcloud(msg: PointCloud2, labels_rgb, labels_np):
    """Add a constant RGB field to a PointCloud2 message."""
    points = list(point_cloud2.read_points(msg, skip_nans=False, field_names=None))
    fields = msg.fields.copy()

    rgb_list = [rgb_to_float(rgb[0], rgb[1], rgb[2]) for rgb in labels_rgb]
    # print(f"len(rgb_list): {len(rgb_list)}")
    # rgb = rgb_to_float(r, g, b)

    # Create new pc msg
    new_rgb_points = [tuple(p) + (rgb,) for p, rgb in zip(points, rgb_list)]
    new_points = [tuple(p) + (label,) for p, label in zip(new_rgb_points, labels_np)]

    # Add RGB field to fields
    rgb_field = PointField(
        name='rgb', 
        offset=msg.point_step, 
        datatype=PointField.FLOAT32, 
        count=1
    )
    fields.append(rgb_field)

    # Add label ID field to fields
    label_id_field = PointField(
        name='label_id',
        offset=msg.point_step + 4,  # after existing fields
        datatype=PointField.UINT16,
        count=1
    )
    fields.append(label_id_field)

    new_msg = point_cloud2.create_cloud(msg.header, fields, new_points)
    new_msg.point_step = msg.point_step + 4 + 2  # float32 = 4 bytes + UINT16 = 2 bytes

    return new_msg

def get_total_message_count_from_metadata(bag_path):
    metadata_path = os.path.join(bag_path, "metadata.yaml")
    if not os.path.exists(metadata_path):
        print("[WARNING] metadata.yaml not found.")
        return None

    with open(metadata_path, 'r') as f:
        try:
            return yaml.safe_load(f).get("rosbag2_bagfile_information", {}).get("message_count")
        except Exception as e:
            print(f"[ERROR] Failed to parse metadata.yaml: {e}")
            return None

def read_and_modify_lidar(bag_path, new_bag_path, lidar_topic, labelpath, max_messages=None):
    lidar_count = 0

    # from collections import deque
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    )

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    lidar_type = type_map.get(lidar_topic)

    if lidar_type is None:
        print(f"[ERROR] Topic {lidar_topic} not found.")
        return

    # msg_type = get_message(lidar_type)
    print(f"\n\nWriting modified bag to: {new_bagpath}\n\n")
    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=new_bag_path, storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    )
    for topic in topic_types:
        writer.create_topic(
            TopicMetadata(name=topic.name, type=topic.type, serialization_format='cdr')
        )

    # Use tqdm with or without total
    total_msgs = get_total_message_count_from_metadata(bag_path)
    progress = tqdm(total=total_msgs, desc="Processing bag")
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg = deserialize_message(data, get_message(type_map[topic]))

        if topic == lidar_topic:
            if max_messages is not None:
                print(f"lidar count: {lidar_count}")
                if lidar_count >= max_messages:
                    break
            
            labels_np = get_semantics(msg, labelpath)
            labels_dict = {label.id: label.color for label in labels}
            labels_rgb = labels2RGB(labels_np, labels_dict)

            lidar_pc_msg = add_rgb_field_to_pointcloud(msg, labels_rgb, labels_np)
            writer.write(topic, serialize_message(lidar_pc_msg), timestamp)
            lidar_count += 1
        else:
            writer.write(topic, serialize_message(msg), timestamp)

        progress.update(1)

    progress.close()


if __name__ == "__main__":
    env = "kittredge_loop"
    robots = [4]

    for robot in robots:
        bagpath = f"/media/donceykong/doncey_ssd_02/datasets/CU_MULTI/ros1_bags/{env}/robot{robot}/robot{robot}_{env}_lidar"
        labelpath = f"/media/donceykong/doncey_ssd_02/datasets/CU_MULTI/ros1_bags/{env}/robot{robot}/label_dir"
        new_bagpath = bagpath + "_with_RGB"
        topic = f"robot{robot}/ouster/points"

        rclpy.init()
        read_and_modify_lidar(bagpath, new_bagpath, topic, labelpath)
        rclpy.shutdown()