#!/usr/bin/env python3

import numpy as np
# from numpy.typing import NDArray
from typing import Tuple

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


def pointcloud_msg_to_numpy(msg: pc2, datatype=np.float32): # -> NDArray[np.float32]:
    """
    """
    # Decode the point cloud-- ours has five float elts:
    field_names = ['x', 'y', 'z', 'intensity', 'reflectivity']
    points = pc2.read_points(msg, field_names=field_names, skip_nans=True)
    pointcloud_numpy_all = np.array(list(points), dtype=datatype)
    
    pointcloud_numpy = pointcloud_numpy_all[:, :4]  # Only keep the x, y, z, intensity values

    return pointcloud_numpy


def parse_gnss_msg(msg: NavSatFix) -> Tuple[float, float, float]:
    """
    Parses a ROS NavSatFix message to extract latitude, longitude, and altitude.

    Args:
        msg (NavSatFix): A ROS NavSatFix message.

    Returns:
        Tuple[float, float, float]: A tuple containing latitude, longitude, and altitude.
    """
    latitude = msg.latitude
    longitude = msg.longitude
    altitude = msg.altitude
    return latitude, longitude, altitude

 
def transform_msg_to_numpy(msg, offset=None):
    """
    """
    translation = np.array([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
    quaternion = np.array([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])

    if offset is not None:
        off_t, off_q = offset
        t += off_t
        w = quaternion[3] * off_q[3] - np.dot(quaternion[:3], off_q[:3])
        v = quaternion[3] * off_q[:3] + off_q[3] * quaternion[:3] + np.cross(quaternion[:3], off_q[:3])
        quaternion[3] = w
        quaternion[:3] = v

    return translation, quaternion


def path_to_numpy(msg): 
    """
    """
    # Preallocate dicts for the poses in the path
    path_quat_ts_data_dict = {}

    # Loop over each pose in the path message
    for pose_msg in msg.poses:
        msg_header_time = f"{pose_msg.header.stamp.to_sec():.20f}"
        odom_quat_flat_numpy = pose_msg_to_numpy(pose_msg)
        path_quat_ts_data_dict[msg_header_time] = odom_quat_flat_numpy

    return path_quat_ts_data_dict


def odom_msg_to_numpy(msg: Odometry): # -> NDArray[np.float32]:
    """
    """
    odom_quat_np = pose_msg_to_numpy(msg.pose)
    return odom_quat_np


def pose_msg_to_numpy(pose_msg: Pose): # -> NDArray[np.float32]: 
    """
    """
    odom_quat_np = np.asarray([pose_msg.pose.position.x, 
                               pose_msg.pose.position.y, 
                               pose_msg.pose.position.z,
                               pose_msg.pose.orientation.x,
                               pose_msg.pose.orientation.y,
                               pose_msg.pose.orientation.z,
                               pose_msg.pose.orientation.w])
    return odom_quat_np


def decode_realsense_image(msg):
    """
    Decode a ROS sensor_msgs/Image message into a numpy array.
    """
    if msg.encoding == "16UC1":  # Monochrome depth image
        dtype = np.uint16
        channels = 1
    elif msg.encoding == "rgb8":  # RGB image
        dtype = np.uint8
        channels = 3
    else:
        raise ValueError(f"Unsupported encoding: {msg.encoding}")

    # Convert the image data to a numpy array and reshape it
    image = np.frombuffer(msg.data, dtype=dtype).reshape(
        msg.height, msg.width, channels
    )

    return image