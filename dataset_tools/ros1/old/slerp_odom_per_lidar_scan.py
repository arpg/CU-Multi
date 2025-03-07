"""

"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R, Slerp

# Load timestamps
ouster_lidar_timestamps_file = "./data/ouster/timestamps.txt"
pose_timestamps_file = "./data/poses/timestamps.txt"
poses_file = "./data/poses/odom.txt"

ouster_timestamps = np.loadtxt(ouster_lidar_timestamps_file)[1000:20000]
pose_timestamps = np.loadtxt(pose_timestamps_file)
poses_data = np.loadtxt(poses_file)

print(f"pose_timestamps[0]: {pose_timestamps[0]}, pose_timestamps[1]: {pose_timestamps[1]}")
# Ensure pose_data is reshaped correctly into 4x4 matrices
poses = poses_data.reshape(-1, 4, 4)

def extract_position_orientation(matrix):
    position = matrix[:3, 3]
    rotation_matrix = matrix[:3, :3]
    quaternion = R.from_matrix(rotation_matrix).as_quat()
    return position, quaternion

positions = []
quaternions = []

for pose in poses:
    pos, quat = extract_position_orientation(pose)
    positions.append(pos)
    quaternions.append(quat)

positions = np.array(positions)
quaternions = np.array(quaternions)

def interpolate_pose(timestamps, positions, quaternions, target_timestamp):
    # Find the interval for interpolation
    idx = np.searchsorted(timestamps, target_timestamp) - 1
    t0, t1 = timestamps[idx], timestamps[idx + 1]
    p0, p1 = positions[idx], positions[idx + 1]
    q0, q1 = quaternions[idx], quaternions[idx + 1]

    # Perform linear interpolation for position
    ratio = (target_timestamp - t0) / (t1 - t0)
    interp_position = (1 - ratio) * p0 + ratio * p1

    # Perform SLERP for orientation
    rotations = R.from_quat([q0, q1])
    print(f"t0: {t0}, t1: {t1}")
    slerp = Slerp([t0, t1], rotations)
    interp_orientation = slerp(target_timestamp).as_quat()

    return interp_position, interp_orientation

# Interpolate poses for all ouster timestamps
interp_position_list = []
interp_orientation_list = []
for ouster_timestamp in ouster_timestamps:
    interp_position, interp_orientation = interpolate_pose(pose_timestamps, positions, quaternions, ouster_timestamp)
    interp_position_list.append(interp_position)
    interp_orientation_list.append(interp_orientation)

interp_positions = np.array(interp_position_list)
interp_orientations = np.array(interp_orientation_list)

# Plot the positions
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], c='b', marker='o', label='Original Positions')
ax.scatter(interp_positions[:, 0], interp_positions[:, 1], interp_positions[:, 2], c='r', marker='^', label='Interpolated Positions')
ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b--', label='Original Path')
ax.plot(interp_positions[:, 0], interp_positions[:, 1], interp_positions[:, 2], 'r--', label='Interpolated Path')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.title('Interpolated Positions Visualization')
plt.show()