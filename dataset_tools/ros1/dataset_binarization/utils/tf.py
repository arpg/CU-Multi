#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as R

def get_quat_orientation(quat):
    """
    Extract position and orientation (as a quaternion) from a 4x4 transformation matrix.
    """
    position = quat[:3]
    rotation_matrix = quat[:]
    quat_rotation = R.from_matrix(rotation_matrix).as_quat()

    return position, quat_rotation

def quat_from_4x4(matrix):
    """
    Extract position and orientation (as a quaternion) from a 4x4 transformation matrix.
    """
    position = matrix[:3, 3]
    rotation_matrix = matrix[:3, :3]
    quaternion = R.from_matrix(rotation_matrix).as_quat()
    
    return position, quaternion

def quat_to_4x4(position, quaternion):
    """
    Create a 4x4 transformation matrix from position and quaternion.
    """
    rotation_matrix = R.from_quat(quaternion).as_matrix()
    pose_matrix = np.eye(4)
    pose_matrix[:3, :3] = rotation_matrix
    pose_matrix[:3, 3] = position
    return pose_matrix

def transform_point_cloud(point_cloud, pose_matrix):
    """
    Apply a 4x4 transformation matrix to a point cloud.
    """
    # Add homogeneous coordinate to the point cloud
    ones = np.ones((point_cloud.shape[0], 1))
    homogeneous_points = np.hstack([point_cloud, ones])

    # Apply transformation
    transformed_points = homogeneous_points.dot(pose_matrix.T)[:, :3]
    return transformed_points