import rosbag
import numpy as np
import open3d as o3d
import struct
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial.transform import Rotation as R, Slerp

def parse_gnss_msg(msg):
    latitude = msg.latitude
    longitude = msg.longitude
    altitude = msg.altitude
    return latitude, longitude, altitude

def decode_realsense_image(msg, display_image=False):
    """
    Decode a ROS sensor_msgs/Image message into a numpy array.
    """
    if msg.encoding == '16UC1': # Monochrome depth image
        dtype = np.uint16
        channels = 1
    elif msg.encoding == 'rgb8': # RGB image
        dtype = np.uint8
        channels = 3
    else:
        raise ValueError(f"Unsupported encoding: {msg.encoding}")
    
    # Convert the image data to a numpy array and reshape it
    image = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)

    if display_image:
        plt.imshow(image,cmap='gray')
        plt.title('Depth Image')
        plt.axis('off')
        plt.show()
    return image

def decode_pointcloud2(cloud, field_names, display_points = False):
    # Read points using the built-in read_points function
    points = pc2.read_points(cloud, field_names=field_names, skip_nans=True)
    
    # Convert the generator to a NumPy array
    points_array = np.array(list(points))
    
    if display_points:
         visualize_pointcloud(pointcloud)

    return points_array

def visualize_pointcloud(points):
    # Visualize point cloud with Open3D
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointcloud[:, :3])
    
    # Normalize intensity values for coloring
    intensity = pointcloud[:, 3] # scale intensity values for better visualization
    intensity_normalized = (intensity - np.min(intensity)) / (np.max(intensity) - np.min(intensity))
    
    # Use intensity as monochrome for colors (R, G, B)
    colors = np.tile(intensity_normalized[:, None], (1, 3))  # repeat normalized intensity for R, G, B channels
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    # Visualize the point cloud
    o3d.visualization.draw_geometries([pcd])

def decode_odom(msg):
    # convert to transformation matrix from x, y, z and quaternion
    odom_mat = np.zeros((4,4))
    odom_mat[0,3] = msg.pose.pose.position.x
    odom_mat[1,3] = msg.pose.pose.position.y
    odom_mat[2,3] = msg.pose.pose.position.z
    odom_mat[3,3] = 1
    # convert quaternion to rotation matrix
    x = msg.pose.pose.orientation.x
    y = msg.pose.pose.orientation.y
    z = msg.pose.pose.orientation.z
    w = msg.pose.pose.orientation.w
    s = x*x + y*y + z*z + w*w

    if s == 0:
        R = np.eye(3)
    else:
        s = 2/s
        X = x*s
        Y = y*s
        Z = z*s
        wX = w*X
        wY = w*Y
        wZ = w*Z
        xX = x*X
        xY = x*Y
        xZ = x*Z
        yY = y*Y
        yZ = y*Z
        zZ = z*Z
        R = np.array([[1-(yY+zZ), xY-wZ, xZ+wY],
                    [xY+wZ, 1-(xX+zZ), yZ-wX],
                    [xZ-wY, yZ+wX, 1-(xX+yY)]])
    odom_mat[:3,:3] = R
    return odom_mat

# Path to your bag file
bag_path = '/home/donceykong/Desktop/doncey_fullnav_rtk2.bag'

# Open the bag file
bag = rosbag.Bag(bag_path)

ouster_points_topic = "/ouster/points"
ouster_scan_num = 1
ouster_timestamp_list = []

depth_image_topic = '/camera/depth/image_rect_raw'
rs_depth_scan_num = 1
rs_depth_timestamp_list = []
rgb_image_topic = "/camera/color/image_raw"
rs_rgb_scan_num = 1
rs_rgb_timestamp_list = []

gnss_1_topic = "/gnss_1/llh_position"
gnss_1_list = []
gnss_1_timestamp_list = []

gnss_2_topic = "/gnss_2/llh_position"
gnss_2_list = []
gnss_2_timestamp_list = []

odom_topic = "/lio_sam/mapping/odometry"
odom_list = []
odom_timestamps_list = []

field_names = ['x', 'y', 'z', 'intensity', 'reflectivity']
# Read messages
for topic, msg, t in bag.read_messages(topics=[ouster_points_topic, odom_topic, gnss_1_topic, gnss_2_topic, depth_image_topic, rgb_image_topic]):
    # print(f"Reading message from: {topic} with timestamp: {msg.header.stamp.secs}.{msg.header.stamp.nsecs}")
    print(f"Bagtime: Time: {t}\n")
    msg_time = f"{msg.header.stamp.secs}.{msg.header.stamp.nsecs}"
    if topic == ouster_points_topic:
        # pointcloud = decode_pointcloud2(msg, field_names, display_points = False)
        # pointcloud.tofile(f"./data/ouster/data/{ouster_scan_num:05d}.bin")
        ouster_scan_num += 1
        ouster_timestamp_list.append(msg_time)
    if topic in [rgb_image_topic, depth_image_topic]:
        # Decode the image message to a numpy array
        image = decode_realsense_image(msg, display_image=False)
        if topic == rgb_image_topic:
            #  image.tofile(f"./data/realsense/rgb_images/data/{rs_rgb_scan_num:05d}.bin")
            rs_rgb_scan_num += 1
            rs_rgb_timestamp_list.append(msg_time)
        elif topic == depth_image_topic:
            #  image.tofile(f"./data/realsense/depth_images/data/{rs_depth_scan_num:05d}.bin")
            rs_depth_scan_num += 1
            rs_depth_timestamp_list.append(msg_time)
    if topic == gnss_1_topic:
        lat, lon, alt = parse_gnss_msg(msg)
        gnss_1_list.append([lat, lon, alt])
        gnss_1_timestamp_list.append(msg_time)
    if topic == gnss_2_topic:
        lat, lon, alt = parse_gnss_msg(msg)
        gnss_2_list.append([lat, lon, alt])
        gnss_2_timestamp_list.append(msg_time)
        #  print(f"lat: {lat}, lon: {lon}, alt: {alt}, Time: {msg.header.stamp.secs}.{msg.header.stamp.nsecs}")
    if topic == odom_topic:
        odom_mat = decode_odom(msg)
        odom_flat = odom_mat.flatten()
        odom_list.append(odom_flat)
        odom_timestamps_list.append(msg_time)

# Save timestamps to files
ouster_timestamps_np = np.array(ouster_timestamp_list)
np.savetxt('data/ouster/timestamps.txt', ouster_timestamps_np, fmt='%s')

rs_rgb_timestamps_np = np.array(rs_rgb_timestamp_list)
rs_depth_timestamps_np = np.array(rs_depth_timestamp_list)
np.savetxt('data/realsense/rgb_images/timestamps.txt', rs_rgb_timestamps_np, fmt='%s')
np.savetxt('data/realsense/depth_images/timestamps.txt', rs_depth_timestamps_np, fmt='%s')

# # Save RTK GPS to txt file
gnss_1_np = np.array(gnss_1_list)
np.savetxt('data/gps/gps_1.txt', gnss_1_np)
gnss_1_timestamps_np = np.array(gnss_1_timestamp_list)
np.savetxt('data/gps/gps_1_timestamps.txt', gnss_1_timestamps_np, fmt='%s')

gnss_2_np = np.array(gnss_2_list)
np.savetxt('data/gps/gps_2.txt', gnss_2_np)
gnss_2_timestamps_np = np.array(gnss_2_timestamp_list)
np.savetxt('data/gps/gps_2_timestamps.txt', gnss_2_timestamps_np, fmt='%s')

# Save odometry to txt file
odom_np = np.array(odom_list)
np.savetxt('data/poses/odom.txt', odom_np)
odom_timestamps_np = np.array(odom_timestamps_list)
np.savetxt('data/poses/timestamps.txt', odom_timestamps_np, fmt='%s')

# Filter pointclouds without associated odometry

# Close the bag file
bag.close()

