import os
import rclpy

from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions
from rclpy.serialization import serialize_message
from rosbag2_py._storage import TopicMetadata
from sensor_msgs.msg import PointCloud2, PointField
from builtin_interfaces.msg import Time

# For TFS
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

# General
import numpy as np
from decimal import Decimal
import struct
from tqdm import tqdm
from collections import namedtuple
from scipy.spatial.transform import Rotation as R, Slerp
# import open3d as o3d

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
    Label("OSM BUILDING", 45, (0, 0, 255)),         # ************ OSM
    Label("OSM ROAD", 46, (255, 0, 0)),             # ************ OSM
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


def convert_unix_sec_to_rostime_msg(timestamp):
    time_msg = Time()
    time_msg.sec = int(timestamp)
    time_msg.nanosec = int((timestamp - time_msg.sec) * int(1e9))
    return time_msg


def read_timestamps(timestamp_path):
    """
    """
    with open(timestamp_path, "r") as ts_file:
        timestamps_sec = [Decimal(line.strip()) for line in ts_file]

    return timestamps_sec
    

def quaternion_pose_to_4x4(trans, quat):
    rotation_matrix = R.from_quat(quat).as_matrix()

    # Create the 4x4 transformation matrix
    transformation_matrix = np.eye(4)  # Start with an identity matrix
    transformation_matrix[:3, :3] = rotation_matrix  # Set the rotation part
    transformation_matrix[:3, 3] = trans  # Set the translation part

    return transformation_matrix


def read_quat_poses(poses_path, pose_ts_path):
    """
    """
    timestamps = read_timestamps(pose_ts_path)
    quat_poses_dict = {}
    with open(poses_path, "r") as file:
        for idx, line in enumerate(file):
            quat_pose = line.strip().split()
            timestamp = timestamps[idx]
            quat_poses_dict[timestamp] = [float(element) for element in quat_pose]
    return quat_poses_dict


def create_semantic_pointcloud2_msg(points, header_time_ns, frame_id):
    """
    Creates a PointCloud2 message from a numpy array of points with RGB data.
    The input `points` is expected to have shape (N, 7), where each row is
    [x, y, z, intensity, r, g, b].
    """
    # Initialize the PointCloud2 message
    msg = PointCloud2()
    msg.header.stamp = header_time_ns
    msg.header.frame_id = frame_id

    # Configure point cloud dimensions
    msg.height = 1  # Unordered point cloud
    msg.width = points.shape[0]

    # Define PointFields (x, y, z, intensity, rgb)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 20  # 16 bytes for x, y, z, intensity + 4 bytes for rgb
    msg.row_step = msg.point_step * points.shape[0]
    msg.is_dense = True

    # Prepare point cloud data
    # Combine RGB components into a single packed float32 value
    rgb = (
        ((points[:, 4]).astype(np.int32) << 16) |  # Red
        ((points[:, 5]).astype(np.int32) << 8) |   # Green
        ((points[:, 6]).astype(np.int32))          # Blue
    )
    rgb_as_float = np.array([struct.unpack('f', struct.pack('I', color))[0] for color in rgb], dtype=np.float32)

    # Combine XYZ, intensity, and RGB into the final data array
    data_array = np.zeros((points.shape[0], 5), dtype=np.float32)
    data_array[:, :4] = points[:, :4].astype(np.float32)  # x, y, z, intensity
    data_array[:, 4] = rgb_as_float  # Pack RGB as float32

    # Convert the array to a binary string (bytes)
    msg.data = data_array.tobytes()

    return msg


def create_pointcloud2_msg(points, header_time_ns, frame_id):
    """
    Creates a PointCloud2 message from a numpy array of points.
    """
    msg = PointCloud2()
    msg.header.stamp = header_time_ns
    msg.header.frame_id = frame_id

    msg.height = 1  # Unordered point cloud
    msg.width = points.shape[0]

    # Point fields (x, y, z, intensity)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16  # 4 fields * 4 bytes/field
    msg.row_step = msg.point_step * points.shape[0]
    msg.is_dense = True

    # Convert numpy array to byte data
    msg.data = np.asarray(points, dtype=np.float32).tobytes()

    return msg


def create_tf_msg(quaternion_tf, header_timestamp, parent_frame_id, child_frame_id):
    """
    """
    transform_msg = TransformStamped()
    transform_msg.header.stamp = header_timestamp
    transform_msg.header.frame_id = parent_frame_id
    transform_msg.child_frame_id = child_frame_id
    transform_msg.transform.translation.x = quaternion_tf[0]
    transform_msg.transform.translation.y = quaternion_tf[1]
    transform_msg.transform.translation.z = quaternion_tf[2]
    transform_msg.transform.rotation.x = quaternion_tf[3]
    transform_msg.transform.rotation.y = quaternion_tf[4]
    transform_msg.transform.rotation.z = quaternion_tf[5]
    transform_msg.transform.rotation.w = quaternion_tf[6]

    return transform_msg


class ROS2BagCreator():
    def __init__(self, dataset_root_dir, environment, robot_number, out_dir):
        # Paths to robot directory
        self.robot_name = f"robot{robot_number}"
        self.sequence_dir = os.path.join(dataset_root_dir, environment, self.robot_name)
        self.lidar_dir = os.path.join(self.sequence_dir, "lidar")

        # Init ROS2 bag writer
        self.writer = SequentialWriter()
        storage_options = StorageOptions(
            uri=f'{out_dir}/{environment}_{self.robot_name}', # Folder where the bag file will be saved
            storage_id='sqlite3'                    # Storage format
        )
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        self.writer.open(storage_options, converter_options)

        print(f"self.sequence_dir: {self.sequence_dir}")

        self.semantic_point_cloud_data_accum = np.array([])

        self.write_all_tfs()
        self.write_all_pointclouds()
        # self.write_all_tfs()

    def write_all_pointclouds(self):
        """ """
        # Create TopicMetadata for the point cloud
        pc_topic_name = f'{self.robot_name}/ouster/points'
        pc_topic_metadata = TopicMetadata(
            name=pc_topic_name,
            type='sensor_msgs/msg/PointCloud2',
            serialization_format='cdr'
        )
        self.writer.create_topic(pc_topic_metadata)

        # Create TopicMetadata for the point cloud
        sem_pc_topic_name = f'{self.robot_name}/ouster/semantic_points'
        sem_pc_topic_metadata = TopicMetadata(
            name=sem_pc_topic_name,
            type='sensor_msgs/msg/PointCloud2',
            serialization_format='cdr'
        )
        self.writer.create_topic(sem_pc_topic_metadata)

        # set frame_id for pointclouds
        pc_frame_id = f'{self.robot_name}_base_link'
        sem_pc_frame_id = f'{self.robot_name}_base_link'

        # To accum points in world frame
        lidar_poses_path = os.path.join(self.lidar_dir, "poses_world.txt")
        quat_poses_list = []
        with open(lidar_poses_path, "r") as file:
            for idx, line in enumerate(file):
                quat_pose = line.strip().split()
                quat_pose_list = [float(element) for element in quat_pose]
                quat_poses_list.append(quat_pose_list)

        # Read lidar timestamps into a list
        lidar_ts_path = os.path.join(self.lidar_dir, "timestamps.txt")
        lidar_timestamps = read_timestamps(lidar_ts_path)

        for idx, timestamp in tqdm(enumerate(lidar_timestamps), total=len(lidar_timestamps), desc="Processing point clouds"):
            pc_bin_file = os.path.join(self.lidar_dir, "pointclouds", f"lidar_pointcloud_{idx}.bin")
            header_time_ns = convert_unix_sec_to_rostime_msg(timestamp)
            msg_time_ns = int(timestamp * int(1e9))
            self.write_single_pointcloud(pc_topic_name, pc_bin_file, header_time_ns, msg_time_ns, pc_frame_id)

            pc_label_bin_file = os.path.join(self.lidar_dir, "labels/gt_osm_labels", f"lidar_pointcloud_{idx}.bin")
            quaternion_tf = quat_poses_list[idx]
            tf = quaternion_pose_to_4x4(quaternion_tf[:3], quaternion_tf[3:])
            # self.write_single_semantic_pointcloud(sem_pc_topic_name, pc_bin_file, pc_label_bin_file, header_time_ns, msg_time_ns, sem_pc_frame_id, tf)
            self.write_single_semantic_pointcloud(sem_pc_topic_name, pc_bin_file, pc_label_bin_file, header_time_ns, msg_time_ns, sem_pc_frame_id)

    def write_single_pointcloud(self, pc_topic_name, pc_file_path, header_time_ns, msg_time, pc_frame_id):
        """ """
        point_cloud_data = np.fromfile(pc_file_path, dtype=np.float32).reshape(-1, 4) # Assuming x, y, z, intensity
        pointcloud_msg = create_pointcloud2_msg(point_cloud_data, header_time_ns, pc_frame_id)
        self.writer.write(pc_topic_name, serialize_message(pointcloud_msg), msg_time)

    def write_single_semantic_pointcloud(self, sem_pc_topic_name, pc_file_path, pc_label_bin_file, header_time_ns, msg_time, sem_pc_frame_id, tf=None):
        """ """
        point_cloud_data = np.fromfile(pc_file_path, dtype=np.float32).reshape(-1, 4) # Assuming x, y, z, intensity
        labels_np = np.fromfile(pc_label_bin_file, dtype=np.int32).reshape(-1)
        labels_dict = {label.id: label.color for label in labels}
        labels_rgb = labels2RGB(labels_np, labels_dict)


        if tf is not None:
            z_limit = 1.0  # Change this to your desired maximum height
            filtered_points = point_cloud_data[point_cloud_data[:, 2] <= z_limit]
            filtered_labels_rgb = labels_rgb[point_cloud_data[:, 2] <= z_limit]
            filtered_labels_np = labels_np[point_cloud_data[:, 2] <= z_limit]

            point_cloud_data = filtered_points
            labels_rgb = filtered_labels_rgb
            labels_np = filtered_labels_np

            # pc_intensity = point_cloud_data[:, 3].flatten()
            pc_xyz = point_cloud_data[:, :3]
            pc_xyz_homogeneous = np.hstack([pc_xyz, np.ones((pc_xyz.shape[0], 1))])
            pc_xyz_tf = np.dot(pc_xyz_homogeneous, tf.T)[:, :3]
            point_cloud_data[:, :3]  = pc_xyz_tf

        # Concatenate point cloud data with RGB values
        # Resulting array has shape (N, 7): [x, y, z, intensity, r, g, b]
        semantic_point_cloud_data = np.hstack([point_cloud_data, labels_rgb])

        desired_semantic_indices = np.where((labels_np == 45) | (labels_np == 46))[0]
        # semantic_point_cloud_data = semantic_point_cloud_data[desired_semantic_indices, :]



        # max_points_per_scan = 500
        # downsampled_indices = np.random.choice(len(semantic_point_cloud_data), max_points_per_scan, replace=False)
        # semantic_point_cloud_data = np.asarray(semantic_point_cloud_data)[downsampled_indices, :]

        # if len(self.semantic_point_cloud_data_accum) > 0:
        #     self.semantic_point_cloud_data_accum = np.concatenate([self.semantic_point_cloud_data_accum, semantic_point_cloud_data])
        # else:
        #     self.semantic_point_cloud_data_accum = semantic_point_cloud_data

        # # max_points = 65000
        # # if len(self.semantic_point_cloud_data_accum) > max_points:
        # #     downsampled_indices = np.random.choice(len(self.semantic_point_cloud_data_accum), max_points, replace=False)
        # #     self.semantic_point_cloud_data_accum = np.asarray(self.semantic_point_cloud_data_accum)[downsampled_indices, :]
        # print(f"len(self.semantic_point_cloud_data_accum): {len(self.semantic_point_cloud_data_accum)}")

        pointcloud_msg = create_semantic_pointcloud2_msg(semantic_point_cloud_data, header_time_ns, sem_pc_frame_id)
        # pointcloud_msg = create_semantic_pointcloud2_msg(self.semantic_point_cloud_data_accum, header_time_ns, sem_pc_frame_id)
        self.writer.write(sem_pc_topic_name, serialize_message(pointcloud_msg), msg_time)

    def get_world_tf_data(self):
        """ """
        lidar_ts_path = os.path.join(self.lidar_dir, "timestamps.txt")
        lidar_poses_path = os.path.join(self.lidar_dir, "poses_world.txt")
        map_poses_dict = read_quat_poses(lidar_poses_path, lidar_ts_path)

        # Set to identity
        for timestamp in map_poses_dict.keys():
            map_poses_dict[timestamp] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

        parent_frame_id = f'world'
        child_frame_id = f'{self.robot_name}_map'
        return map_poses_dict, parent_frame_id, child_frame_id
    
    def get_lidar_tf_data(self):
        """ """
        lidar_ts_path = os.path.join(self.lidar_dir, "timestamps.txt")
        lidar_poses_path = os.path.join(self.lidar_dir, "poses_world.txt")
        lidar_poses_dict = read_quat_poses(lidar_poses_path, lidar_ts_path)
        parent_frame_id = f'{self.robot_name}_map'
        child_frame_id = f'{self.robot_name}_base_link'
        return lidar_poses_dict, parent_frame_id, child_frame_id

    def write_all_tfs(self):
        """ """
        # Create TopicMetadata for the TF topic
        tf_topic_name = '/tf'
        tf_topic_metadata = TopicMetadata(
            name=tf_topic_name,
            type='tf2_msgs/msg/TFMessage',
            serialization_format='cdr'
        )
        self.writer.create_topic(tf_topic_metadata)
        
        all_tf_pose_data = []
        all_tf_pose_data.append(self.get_lidar_tf_data())   # Lidar tfs
        all_tf_pose_data.append(self.get_world_tf_data())   # Robot Map Frame TF

        for tf_pose_data in all_tf_pose_data:
            self.write_tfs(tf_topic_name, tf_pose_data)
    
    def write_tfs(self, tf_topic_name, tf_pose_data):
        """ """
        poses_dict, parent_frame_id, child_frame_id = tf_pose_data
        for timestamp in tqdm(poses_dict.keys(), total=len(poses_dict.keys()), desc="Processing tfs for pointclouds"):
            header_timestamp = convert_unix_sec_to_rostime_msg(timestamp)
            msg_time_ns = int(timestamp * int(1e9))
            tf_msg = create_tf_msg(poses_dict[timestamp], header_timestamp, parent_frame_id, child_frame_id)
            self.write_single_tf(tf_topic_name, tf_msg, msg_time_ns)

    def write_single_tf(self, tf_topic_name, transform, msg_time_ns):
        """ """
        tf_msg = TFMessage()
        tf_msg.transforms.append(transform)
        self.writer.write(tf_topic_name, serialize_message(tf_msg), msg_time_ns)


def main(args=None):
    rclpy.init(args=args)

    # Path to root of CU-MULTI Dataset directory
    dataset_root_dir = '/media/donceykong/donceys_data_ssd/datasets/CU_MULTI/data/'
    environment = "kittredge_loop"
    robot_num = "4"
    out_dir = "/media/donceykong/doncey_ssd_011/datasets/CU_MULTI/ros2_bags" #f"{dataset_root_dir}"

    ROS2BagCreator(dataset_root_dir, environment, robot_num, out_dir)


if __name__ == '__main__':
    main()
