#!/usr/bin/env python3

import os
import numpy as np
np.set_printoptions(suppress=True)
from array import array
import rosbag
from sensor_msgs.msg import CameraInfo
import json 

# Internal
from utils.rosmsg_to_numpy import *

class BagParser:
    def __init__(self, log_paths_dict, robot_name):
        # Set paths for run
        for key, value in log_paths_dict.items():
            setattr(self, key, value)

        # Set topics
        self.ouster_points_topic = f"{robot_name}/ouster/points"
        self.odom_topic = f"{robot_name}/lio_sam/mapping/odometry"
        self.path_topic = f"{robot_name}/lio_sam/mapping/path" 
        self.gnss_1_topic = f"{robot_name}/gnss_1/llh_position"
        self.gnss_2_topic = f"{robot_name}/gnss_2/llh_position"

        # self.gnss_ekf_topic = f"{robot_name}/ekf/llh_position"
        # self.imu_data_topic = f"{robot_name}/imu/data"
        # self.gnss_ekf_heading_topic = f"{robot_name}/ekf/dual_antenna_heading"
        # self.camera_depth_image_topic = f"{robot_name}/camera/depth/image_rect_raw"
        # self.camera_depth_info_topic = f"{robot_name}/camera/depth/camera_info"
        # self.camera_rgb_image_topic = f"{robot_name}/camera/color/image_raw"
        # self.camera_rgb_info_topic = f"{robot_name}/camera/color/camera_info"
        # self.transforms_topic = "/tf"
        # self.static_transforms_topic = "/tf_static"
        
        # Dictionary that maps topics to their respective handling functions. Keys can be used as wanted topics.
        self.topics_handlers_dict = {
            self.ouster_points_topic: self.handle_ouster_pointcloud,
            self.odom_topic: self.handle_odometry,
            self.path_topic: self.handle_path, 
            self.gnss_1_topic: self.handle_gnss1_gps, 
            self.gnss_2_topic: self.handle_gnss2_gps, 
            # self.imu_data_topic: self.handle_imu_data,
            # self.camera_rgb_image_topic: self.handle_rgb_image,
            # self.camera_rgb_info_topic:self.handle_rgb_cam_info, 
            # self.camera_depth_image_topic: self.handle_depth_image,
            # self.camera_depth_info_topic: self.handle_depth_cam_info, 
        }

        # Initialize number of data to 0 so that it corresponds with line in timestamp text file
        self.lidar_pc_num = 0
        # self.camera_rgb_num = 0
        # self.camera_depth_num = 0

        # Initialize dictionaries for timestamps and data, where dict[timestamp] = data
        self.ouster_ts_index_dict = {}    
        self.gnss1_ts_data_dict = {}  
        self.gnss2_ts_data_dict = {}   
        self.odom_ts_data_dict = {}
        self.fin_path_ts_data_dict = {}
        # self.imu_ts_data_dict = {} 
        # self.camera_depth_ts_index_dict = {}   
        # self.camera_rgb_ts_index_dict = {}

        self.path_msg = None
        # self.rgb_info_initted = False 
        # self.rgb_info = None 
        # self.depth_info_initted = False  
        # self.depth_info =  None 

    # def handle_rgb_cam_info(self,msg,msg_header_time): 
    #     if not self.rgb_info_initted: 
    #         self.rgb_info_initted = True 
    #         self.rgb_info = cam_info_to_json(msg)  

    # def handle_rgb_image(self, msg, msg_header_time):
    #     image = utils.image_msg_to_numpy(msg=msg)

    #     # Save rgb image
    #     filename = f"{self.camera_rgb_path}/unsorted_camera_rgb_image_{self.camera_rgb_num}.bin"
    #     image.tofile(filename)

    #     # Store timestamp and index
    #     self.camera_rgb_ts_index_dict[msg_header_time] = self.camera_rgb_num
    #     self.camera_rgb_num += 1

    # def handle_depth_cam_info(self,msg,msg_header_time): 
    #     if not self.depth_info_initted: 
    #         self.depth_info_initted = True 
    #         self.depth_info = utils.cam_info_to_json(msg) 

    # def handle_depth_image(self, msg, msg_header_time):
    #     image = utils.image_msg_to_numpy(msg, datatype=np.uint16, num_channels=1)

    #     # Save depth image
    #     filename = f"{self.camera_depth_path}/unsorted_camera_depth_image_{self.camera_depth_num}.bin"
    #     image.tofile(filename)

    #     # Store timestamp and index
    #     self.camera_depth_ts_index_dict[msg_header_time] = self.camera_depth_num
    #     self.camera_depth_num += 1

    # def handle_imu_data(self, msg, msg_header_time):
    #     # Decode IMU
    #     imu_numpy = imu_msg_to_numpy(msg)
    #     self.imu_ts_data_dict[msg_header_time] = imu_numpy[:6]  # Do not save orientation data

    def handle_path(self, msg, msg_header_time=None): 
        self.path_msg = msg

    def handle_odometry(self, msg, msg_header_time):
        odom_quat_flat = odom_msg_to_numpy(msg)
        self.odom_ts_data_dict[msg_header_time] = odom_quat_flat
        # gt_data_file.write(str(p_x) + ' ' + str(p_y) + ' ' + str(p_z) + ' ' + str(q_x) + ' ' + str(q_y) + ' ' + str(q_z) + ' ' + str(q_w) + '\n')

    def handle_gnss1_gps(self, msg, msg_header_time):
        latlonalt = parse_gnss_msg(msg)
        self.gnss1_ts_data_dict[msg_header_time] = latlonalt

    def handle_gnss2_gps(self, msg, msg_header_time):
        latlonalt = parse_gnss_msg(msg)
        self.gnss2_ts_data_dict[msg_header_time] = latlonalt

    def handle_ouster_pointcloud(self, msg, msg_header_time):
        pointcloud = pointcloud_msg_to_numpy(msg)

        # Save point cloud
        pointcloud_filename = f"{self.lidar_pc_bin_path}/unsorted_lidar_pointcloud_{self.lidar_pc_num}.bin"
        pointcloud.tofile(pointcloud_filename)

        # Store timestamp and index
        self.ouster_ts_index_dict[msg_header_time] = self.lidar_pc_num
        self.lidar_pc_num += 1

    def read_bag(self, rosbag_path):
        # Open specified rosbag file
        bag = rosbag.Bag(rosbag_path)

        for topic, msg, bag_time in bag.read_messages(self.topics_handlers_dict.keys()):
            # print(f"Processing message from topic: {topic}")
            # msg_time = f"{msg.header.stamp.secs}.{msg.header.stamp.nsecs}"
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                msg_header_time = f"{msg.header.stamp.to_sec():.20f}"
            elif hasattr(msg, 'transforms'):
                msg_header_time = None #f"{msg.transforms.header.stamp.to_sec()}"

            # Dispatch message to the corresponding handler function
            self.topics_handlers_dict[topic](msg, msg_header_time)

        # Assign final path attributes after bag has been completely read
        self.fin_path_ts_data_dict = path_to_numpy(self.path_msg)

        # Close the bag file
        bag.close()

    def write_data_to_files(self):
        # Helper function to handle timestamps, renaming, and saving data
        def save_and_rename(timestamp_file_prefix, ts_index_dict, timestamp_path, data_path, filename_prefix):
            timestamps_np = np.array(sorted(ts_index_dict.keys()))
            np.savetxt(f'{timestamp_path}/{timestamp_file_prefix}timestamps.txt', timestamps_np, fmt='%s')

            for idx, timestamp in enumerate(timestamps_np):
                original_index = ts_index_dict[timestamp]
                new_index = idx + 1
                original_filename = f"{data_path}/unsorted_{filename_prefix}_{original_index}.bin"
                new_filename = f"{data_path}/{filename_prefix}_{new_index}.bin"
                os.rename(original_filename, new_filename)
        
        # # Write and rename CAMERA RGB data
        # save_and_rename("rgb_", self.camera_rgb_ts_index_dict, self.camera_path, self.camera_rgb_path, "camera_rgb_image")
        
        # # Write and rename CAMERA DEPTH data
        # save_and_rename("depth_", self.camera_depth_ts_index_dict, self.camera_path, self.camera_depth_path, "camera_depth_image")
        
        # Write and rename LIDAR data
        save_and_rename("", self.ouster_ts_index_dict, self.lidar_path, self.lidar_pc_bin_path, "lidar_pointcloud")

        # # Write IMU timestamps and data
        # imu_timestamps_np = np.array(sorted(self.imu_ts_data_dict.keys()))
        # np.savetxt(f'{self.imu_path}/timestamps.txt', imu_timestamps_np, fmt='%s')

        # imu_data_np = np.array([self.imu_ts_data_dict[timestamp] for timestamp in imu_timestamps_np])
        # np.savetxt(f'{self.imu_path}/imu_data.txt', imu_data_np)

        # Save RTK GPS to txt file
        gnss_1_timestamps_np = np.array(sorted(self.gnss1_ts_data_dict.keys()))
        np.savetxt(f"{self.gps_path}/gnss_1_timestamps.txt", gnss_1_timestamps_np, fmt="%s")
        gnss_1_np = np.array([self.gnss1_ts_data_dict[timestamp] for timestamp in gnss_1_timestamps_np])
        np.savetxt(f"{self.gps_path}/gnss_1_data.txt", gnss_1_np)

        gnss_2_timestamps_np = np.array(sorted(self.gnss2_ts_data_dict.keys()))
        np.savetxt(f"{self.gps_path}/gnss_2_timestamps.txt", gnss_2_timestamps_np, fmt="%s")
        gnss_2_np = np.array([self.gnss2_ts_data_dict[timestamp] for timestamp in gnss_2_timestamps_np])
        np.savetxt(f"{self.gps_path}/gnss_2_data.txt", gnss_2_np)

        # Write odometry timestamps and data
        odom_timestamps_np = np.array(sorted(self.odom_ts_data_dict.keys()))
        np.savetxt(f'{self.poses_path}/odom_timestamps.txt', odom_timestamps_np, fmt='%s')
        odom_quat_np = np.array([self.odom_ts_data_dict[timestamp] for timestamp in odom_timestamps_np])
        np.savetxt(f'{self.poses_path}/groundtruth_odom.txt', odom_quat_np) 

        # Write final path timestamps and data to txt files
        fin_path_timestamps_np = np.array(sorted(self.fin_path_ts_data_dict.keys()))
        np.savetxt(f'{self.poses_path}/path_timestamps.txt', fin_path_timestamps_np, fmt='%s')
        fin_path_quat_np = np.array([self.fin_path_ts_data_dict[timestamp] for timestamp in fin_path_timestamps_np])
        np.savetxt(f'{self.poses_path}/groundtruth_path.txt', fin_path_quat_np) 

        # # Save Camera calib info 
        # with open(self.camera_path + '/rgb_cam_info.json','w') as f: 
        #     json.dump(self.rgb_info,f,indent=4) 

        # with open(self.camera_path + '/depth_cam_info.json','w') as f:
        #     json.dump(self.depth_info,f,indent=4)   