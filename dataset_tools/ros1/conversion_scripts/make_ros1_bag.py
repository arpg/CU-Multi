import os
import rospy
import rosbag
import numpy as np
import struct
from decimal import Decimal
from collections import namedtuple
from tqdm import tqdm
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

# --- Label Handling ---
Label = namedtuple("Label", ["name", "id", "color"])

labels = [
    Label("OSM BUILDING", 45, (0, 0, 255)),
    Label("OSM ROAD", 46, (255, 0, 0)),
    Label("building", 50, (0, 100, 0)),
]

def labels2RGB(label_ids, labels_dict):
    rgb_array = np.zeros((len(label_ids), 3), dtype=float)
    for idx, label_id in enumerate(label_ids):
        if label_id in labels_dict:
            rgb_array[idx] = np.array(labels_dict[label_id])
    return rgb_array

# --- Time Conversion ---
def convert_unix_sec_to_rostime_msg(timestamp):
    if isinstance(timestamp, Decimal):
        timestamp = float(timestamp)
    secs = int(timestamp)
    nsecs = int((timestamp - secs) * 1e9)
    return rospy.Time(secs, nsecs)

# def convert_unix_sec_to_rostime_msg(timestamp):
#     time_msg = rospy.Time()
#     time_msg.secs = int(timestamp)
#     print("FUCK")
#     time_msg.nanosec = int((timestamp - time_msg.secs) * 1e9)
#     return time_msg #rospy.Time(secs, nsecs)

# --- PointCloud2 Creation ---
def create_pointcloud2_msg(points, stamp, frame_id):
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = points.shape[0]
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * points.shape[0]
    msg.is_dense = True
    msg.data = np.asarray(points, dtype=np.float32).tobytes()
    return msg

def create_semantic_pointcloud2_msg(points, stamp, frame_id):
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = points.shape[0]
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 20
    msg.row_step = msg.point_step * points.shape[0]
    msg.is_dense = True

    rgb = (
        ((points[:, 4]).astype(np.int32) << 16) |
        ((points[:, 5]).astype(np.int32) << 8) |
        (points[:, 6]).astype(np.int32)
    )
    rgb_as_float = np.array([struct.unpack('f', struct.pack('I', c))[0] for c in rgb], dtype=np.float32)

    data_array = np.zeros((points.shape[0], 5), dtype=np.float32)
    data_array[:, :4] = points[:, :4]
    data_array[:, 4] = rgb_as_float
    msg.data = data_array.tobytes()
    return msg

# --- TF ---
def create_tf_msg(tf_data, stamp, parent_frame, child_frame):
    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = parent_frame
    tf.child_frame_id = child_frame
    tf.transform.translation.x = tf_data[0]
    tf.transform.translation.y = tf_data[1]
    tf.transform.translation.z = tf_data[2]
    tf.transform.rotation.x = tf_data[3]
    tf.transform.rotation.y = tf_data[4]
    tf.transform.rotation.z = tf_data[5]
    tf.transform.rotation.w = tf_data[6]
    return tf

# --- Utilities ---
def read_timestamps(path):
    with open(path, 'r') as f:
        return [Decimal(line.strip()) for line in f]

def read_quat_poses(poses_path, ts_path):
    timestamps = read_timestamps(ts_path)
    pose_dict = {}
    with open(poses_path, 'r') as f:
        for idx, line in enumerate(f):
            pose = [float(x) for x in line.strip().split()]
            pose_dict[timestamps[idx]] = pose
    return pose_dict

# --- Main Bag Creator ---
class ROS1BagCreator:
    def __init__(self, dataset_root_dir, environment, robot_number, out_dir):
        self.robot_name = f"robot{robot_number}"
        self.sequence_dir = os.path.join(dataset_root_dir, environment, self.robot_name)
        self.lidar_dir = os.path.join(self.sequence_dir, "lidar")
        bag_path = os.path.join(out_dir, f"{environment}_{self.robot_name}.bag")
        self.bag = rosbag.Bag(bag_path, 'w')
        self.labels_dict = {label.id: label.color for label in labels}
        self.write_all_tfs()
        self.write_all_pointclouds()

    def write_all_pointclouds(self):
        pose_path = os.path.join(self.lidar_dir, "poses_world.txt")
        ts_path = os.path.join(self.lidar_dir, "timestamps.txt")
        poses = read_quat_poses(pose_path, ts_path)
        timestamps = read_timestamps(ts_path)

        for idx, timestamp in tqdm(enumerate(timestamps), total=len(timestamps), desc="PointClouds"):
            pc_file = os.path.join(self.lidar_dir, "pointclouds", f"lidar_pointcloud_{idx}.bin")
            label_file = os.path.join(self.lidar_dir, "labels", f"lidar_pointcloud_{idx}.bin")
            stamp = convert_unix_sec_to_rostime_msg(timestamp)

            points = np.fromfile(pc_file, dtype=np.float32).reshape(-1, 4)
            labels_np = np.fromfile(label_file, dtype=np.int32).reshape(-1)
            labels_rgb = labels2RGB(labels_np, self.labels_dict)
            sem_points = np.hstack([points, labels_rgb])

            mask = np.where((labels_np == 45) | (labels_np == 46))[0]
            sem_points = sem_points[mask, :]

            msg = create_semantic_pointcloud2_msg(sem_points, stamp, f"{self.robot_name}_base_link")
            self.bag.write(f"/{self.robot_name}/ouster/semantic_points", msg, stamp)

            raw_msg = create_pointcloud2_msg(points, stamp, f"{self.robot_name}_base_link")
            self.bag.write(f"/{self.robot_name}/ouster/points", raw_msg, stamp)

    def write_all_tfs(self):
        ts_path = os.path.join(self.lidar_dir, "timestamps.txt")
        poses = read_quat_poses(os.path.join(self.lidar_dir, "poses_world.txt"), ts_path)

        for timestamp, pose in tqdm(poses.items(), desc="TFs"):
            stamp = convert_unix_sec_to_rostime_msg(timestamp)
            tf = create_tf_msg(pose, stamp, f"{self.robot_name}_map", f"{self.robot_name}_base_link")
            tf_msg = TFMessage(transforms=[tf])
            self.bag.write("/tf", tf_msg, stamp)

    def __del__(self):
        self.bag.close()

# --- Main ---
if __name__ == '__main__':
    rospy.init_node("ros1_bag_creator")
    dataset_root_dir = '/root/Datasets/cu_multi'
    environment = "kittredge_loop"
    robot_number = "1"
    out_dir = dataset_root_dir
    ROS1BagCreator(dataset_root_dir, environment, robot_number, out_dir)
