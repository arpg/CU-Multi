#!/usr/bin/env python3

import numpy as np
# from numpy.typing import NDArray
from typing import Tuple

from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2
from pypcd4 import PointCloud as pc_pypcd

from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

def pointcloud_msg_to_numpy(msg):
    pc = pc_pypcd.from_msg(msg)
    print(pc.pc_data['t'])

# def pointcloud_msg_to_numpy(msg, datatype=np.float32):
#     # List out the fields
#     print("PointCloud2 has these fields:")
#     for f in msg.fields:
#         # Map the numeric datatype back to its name
#         dt_name = {v: k for k, v in PointField.__dict__.items() 
#                    if k.startswith("DATATYPE_")}.get(f.datatype, f.datatype)
#         print(f"  • {f.name}: offset={f.offset}, datatype={dt_name}, count={f.count}")

#     # Now read the points, matching that order
#     field_names = [f.name for f in msg.fields]
#     for pt in pc2.read_points(msg, field_names=field_names, skip_nans=False):
#         # pt is a tuple whose elements line up with msg.fields
#         for name, val in zip(field_names, pt):
#             print(f"    {name} = {val}")
#         break  # just print the first point

def pointcloud_msg_to_numpy(msg: pc2, datatype=np.float32): # -> NDArray[np.float32]:
    """ Returns np array consisting of lidar pointcloud.
    """
    # Decode the point cloud-- ours has five float elts:
    # field_names = None # Will then save all fields
    field_names = ['x', 'y', 'z', 'intensity', 'reflectivity']#, 't']
    points = pc2.read_points(msg, field_names=field_names, skip_nans=False)
    # for pt in points:
    #     print(pt)

    pointcloud_numpy = np.array(list(points), dtype=datatype)

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

def imu_msg_to_numpy(msg):
    orientation = msg.orientation           # x, y, z, w
    angular_vel = msg.angular_velocity      # x, y, z
    linear_accel = msg.linear_acceleration  # x, y, z

    # Flatten and concatenate all IMU components
    imu_values = np.asarray([orientation.x, 
                             orientation.y, 
                             orientation.z, 
                             orientation.w,
                             angular_vel.x,
                             angular_vel.y, 
                             angular_vel.z,
                             linear_accel.x, 
                             linear_accel.y, 
                             linear_accel.z], dtype=np.float64)

    print(f"\nimu_values:\n{imu_values}\n")
    return imu_values
    # imu_data_str = " ".join(f"{v}" for v in imu_values)