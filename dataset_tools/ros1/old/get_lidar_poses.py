import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
from decimal import Decimal
import open3d as o3d

def extract_position_orientation(matrix):
    """
    Extract position and orientation (as a quaternion) from a 4x4 transformation matrix.
    """
    position = matrix[:3, 3]
    rotation_matrix = matrix[:3, :3]
    quaternion = R.from_matrix(rotation_matrix).as_quat()
    return position, quaternion

def interpolate_pose(timestamps, positions, quaternions, target_timestamp):
    """
    Interpolate the pose (position and orientation) for a given target timestamp.
    """
    # Find the interval for interpolation
    idx = np.searchsorted(timestamps, target_timestamp) - 1

    # Check for out-of-bounds and assign default values if necessary
    if idx < 0 or idx >= len(timestamps) - 1:
        return None, None  # Or use -1, -1 for both position and orientation if preferred

    t0, t1 = timestamps[idx], timestamps[idx + 1]
    p0, p1 = positions[idx], positions[idx + 1]
    q0, q1 = quaternions[idx], quaternions[idx + 1]

    # Perform linear interpolation for position
    ratio = float((target_timestamp - t0) / (t1 - t0))  # Convert Decimal to float for ratio
    print(f"t0: {t0}, t1: {t1}, ratio: {ratio}")
    interp_position = (1 - ratio) * p0 + ratio * p1
    # slerp = Slerp([float(t0), float(t1)], positions)
    # interp_position = slerp(float(target_timestamp))

    # Perform SLERP for orientation
    rotations = R.from_quat([q0, q1])
    slerp = Slerp([float(t0), float(t1)], rotations)  # Convert Decimals to floats for Slerp
    interp_orientation = slerp(float(target_timestamp)).as_quat()

    return interp_position, interp_orientation

def create_4x4_pose_matrix(position, quaternion):
    """
    Create a 4x4 transformation matrix from position and quaternion.
    """
    rotation_matrix = R.from_quat(quaternion).as_matrix()
    pose_matrix = np.eye(4)
    pose_matrix[:3, :3] = rotation_matrix
    pose_matrix[:3, 3] = position
    return pose_matrix

def save_pose_as_bin(pose_matrix, filename):
    """
    Save a 4x4 pose matrix as a binary file.
    """
    pose_matrix.astype(np.float32).tofile(filename)

# Load timestamps
ouster_lidar_timestamps_file = "./data/ouster/timestamps.txt"
pose_timestamps_file = "./data/poses/timestamps.txt"
poses_file = "./data/poses/odom.txt"

pose_timestamps_str = np.loadtxt(pose_timestamps_file, dtype=str)
pose_timestamps = [Decimal(ts) for ts in pose_timestamps_str]   # Convert to Decimal for high precision
poses_data = np.loadtxt(poses_file)

# Ensure pose_data is reshaped correctly into 4x4 matrices
poses = poses_data.reshape(-1, 4, 4)

positions = []
quaternions = []

for pose in poses:
    pos, quat = extract_position_orientation(pose)
    positions.append(pos)
    quaternions.append(quat)

positions = np.array(positions)
quaternions = np.array(quaternions)

min_timestamp = min(pose_timestamps)
max_timestamp = max(pose_timestamps)

print(f"pose_timestamps[0]: {pose_timestamps[0]}, pose_timestamps[-1]: {pose_timestamps[-1]}")
print(f"min_timestamp: {min_timestamp}, max_timestamp: {max_timestamp}")

# Load and filter ouster timestamps
ouster_timestamps_str = np.loadtxt(ouster_lidar_timestamps_file, dtype=str)
ouster_timestamps = [Decimal(ts) for ts in ouster_timestamps_str]   # Convert to Decimal for high precision

# Filter ouster timestamps to find those within the min-max range
ouster_filtered_timestamps = [ts for ts in ouster_timestamps if min_timestamp <= ts <= max_timestamp]

# Interpolate poses for all ouster timestamps
interp_position_list = []
interp_orientation_list = []

import os
# Directory to save the pose matrices
output_directory = "./data/ouster/poses/"
os.makedirs(output_directory, exist_ok=True)

for ouster_timestamp in ouster_filtered_timestamps:
    print(f"Interpolating for ouster timestamp: {ouster_timestamp}")
    interp_position, interp_orientation = interpolate_pose(pose_timestamps, positions, quaternions, ouster_timestamp)
    if interp_position is not None and interp_orientation is not None:
        interp_position_list.append(interp_position)
        interp_orientation_list.append(interp_orientation)
        
        # Create a 4x4 pose matrix
        pose_matrix = create_4x4_pose_matrix(interp_position, interp_orientation)
        
        # Save the pose matrix as a binary file with underscores in the timestamp
        timestamp_str = str(ouster_timestamp).replace('.', '_')
        bin_filename = os.path.join(output_directory, f"{timestamp_str}.bin")
        save_pose_as_bin(pose_matrix, bin_filename)
        print(f"Saved interpolated pose as {bin_filename}")

# Convert lists to numpy arrays for visualization
interp_positions = np.array(interp_position_list)
interp_orientations = np.array(interp_orientation_list)

def visualize_positions(original_positions, interpolated_positions):
    # Identify non-matching interpolated points
    interpolated_set = set(map(tuple, interpolated_positions))
    original_set = set(map(tuple, original_positions))
    unique_interpolated_positions = np.array([pos for pos in interpolated_set if pos not in original_set])

    # Create Open3D point clouds for unique interpolated positions
    unique_interpolated_pcd = o3d.geometry.PointCloud()
    unique_interpolated_pcd.points = o3d.utility.Vector3dVector(unique_interpolated_positions)
    unique_interpolated_pcd.paint_uniform_color([1, 0, 0])  # Red for unique interpolated

    # Set points for original positions
    original_pcd = o3d.geometry.PointCloud()
    original_pcd.points = o3d.utility.Vector3dVector(original_positions)
    original_pcd.paint_uniform_color([0, 0, 1])  # Blue for original

    # Visualize the unique interpolated points
    o3d.visualization.draw_geometries([unique_interpolated_pcd, original_pcd],
                                      window_name='Unique Interpolated Positions',
                                      width=800, height=600)
    
# Visualize the original and interpolated positions
visualize_positions(positions, interp_positions)
