import rosbag
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
import open3d as o3d
import os
import rosbag
import re

def get_bag_duration(bag_path):
    bag = rosbag.Bag(bag_path)
    start_time = bag.get_start_time()
    end_time = bag.get_end_time()
    duration = end_time - start_time
    bag.close()
    return duration

def decode_odom(msg):
    # Convert to transformation matrix from x, y, z and quaternion
    odom_mat = np.zeros((4, 4))
    odom_mat[0, 3] = msg.pose.pose.position.x
    odom_mat[1, 3] = msg.pose.pose.position.y
    odom_mat[2, 3] = msg.pose.pose.position.z
    odom_mat[3, 3] = 1
    
    # Convert quaternion to rotation matrix
    x = msg.pose.pose.orientation.x
    y = msg.pose.pose.orientation.y
    z = msg.pose.pose.orientation.z
    w = msg.pose.pose.orientation.w
    s = x*x + y*y + z*z + w*w

    if s == 0:
        R = np.eye(3)
    else:
        s = 2 / s
        X = x * s
        Y = y * s
        Z = z * s
        wX = w * X
        wY = w * Y
        wZ = w * Z
        xX = x * X
        xY = x * Y
        xZ = x * Z
        yY = y * Y
        yZ = y * Z
        zZ = z * Z
        R = np.array([[1-(yY+zZ), xY-wZ, xZ+wY],
                      [xY+wZ, 1-(xX+zZ), yZ-wX],
                      [xZ-wY, yZ+wX, 1-(xX+yY)]])
    odom_mat[:3, :3] = R
    return odom_mat


def visualize_trajectory_with_orientation(odom_list):
    # Create open3d LineSet for trajectory visualization
    points = np.array([(odom[0, 3], odom[1, 3], odom[2, 3]) for odom in odom_list])
    lines = [[i, i + 1] for i in range(len(points) - 1)]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.paint_uniform_color([0, 0, 1])  # Set line color to blue

    # Create coordinate frames for orientation visualization
    frames = []
    for odom in odom_list:
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
        frame.transform(odom)
        frames.append(frame)

    # Visualize using open3d
    o3d.visualization.draw_geometries([line_set, *frames], window_name='Odometry Trajectory with Orientation',
                                      zoom=0.5, front=[0.5, -0.5, -0.5], lookat=[0, 0, 0], up=[0, 1, 0])
    
def get_bag_path_length(bag_path, odom_topic):
    # Open the bag file
    odom_list = []
    odom_mat_list = []
    bag = rosbag.Bag(bag_path)
    for topic, msg, t in bag.read_messages(topics=odom_topic):
        odom_mat = decode_odom(msg)
        odom_flat = odom_mat.flatten()
        odom_list.append(odom_flat)
        odom_mat_list.append(odom_mat)

    # Extract positions from the odometry data
    positions = [(odom[3], odom[7], odom[11]) for odom in odom_list]

    # Calculate the total path length
    total_path_length = 0.0
    for i in range(1, len(positions)):
        # Euclidean distance between consecutive positions
        p1 = np.array(positions[i-1])
        p2 = np.array(positions[i])
        distance = np.linalg.norm(p2 - p1)
        total_path_length += distance

    bag.close()
    return total_path_length, odom_mat_list

def get_seq_stats(directory_root_path, directories, odom_topic):
    total_duration = 0
    total_distance_covered = 0
    for directory in directories:
        directory_path = os.path.join(directory_root_path, directory)
        if os.path.exists(directory_path) and os.path.isdir(directory_path):
            sequence_duration = 0
            sequence_path_length = 0
            files = os.listdir(directory_path)

            # Sort files based on the numerical value before the ".bag" extension
            files.sort(key=lambda x: int(re.search(r'(\d+)\.bag$', x).group(1)) if re.search(r'(\d+)\.bag$', x) else 0)

            print(f"\nSequence: {directory}")
            for file in files:
                if file.endswith(".bag"):
                    bag_path = os.path.join(directory_path, file)
                    
                    duration = get_bag_duration(bag_path)
                    sequence_duration += duration
                    minutes, seconds = divmod(int(duration), 60)

                    path_length, odom_list = get_bag_path_length(bag_path, odom_topic)
                    sequence_path_length += path_length
                    # visualize_trajectory_with_orientation(odom_list)

                    print(f"    - {file}: {minutes}:{seconds:02d} min:sec, path len: {path_length:0.2f} meters.")
                    total_duration += duration
                    total_distance_covered += path_length
            seq_minutes, seq_seconds = divmod(int(sequence_duration), 60)
            print(f"Total duration for {directory} sequence: {seq_minutes}:{seq_seconds:02d} min:sec")
            print(f"Total path length for {directory} sequence: {sequence_path_length:0.2f} meters.")
    return total_duration, total_distance_covered

# Define the directories to search for bag files
directory_path = "/media/donceykong/doncey_ssd_01/coloradar_plus_dataset/bags"
directories = [
    "ec_courtyard_liosam",
    # "c4c_garage",
    # "regent_garage",
    # "ec_hallways",
    # "irl",
]
odom_topic = "/lio_sam/mapping/odometry"
total_duration_seconds, total_distance_covered = get_seq_stats(directory_path, directories, odom_topic)
total_minutes, total_seconds = divmod(int(total_duration_seconds), 60)

print(f"\nTotal duration dataset: {total_minutes}:{total_seconds:02d} min:sec")
print(f"Total distance covered dataset: {total_distance_covered:0.2f} meters.")