#!/usr/bin/env python3

import os
import numpy as np
import shutil

# Internal Imports
from bag_parser import BagParser

class DatasetToBin:
    def __init__(self, lidar2osm_config_dict = None):
        self.lidar2osm_config_dict = lidar2osm_config_dict 
        
    def convert(self):
        """_Converts bags specified in config file to KITTI-style unstructured dataset._
        """
        # Extract data from runs in each sequence specified in config
        for seq_name in self.lidar2osm_config_dict["sequences"]:
            print(f"\n\nSequence: {seq_name}")
            sequence = self.lidar2osm_config_dict["sequences"][f"{seq_name}"]
            for robot_number in sequence["robots"]:
                self.bag_to_kitti(seq_name, robot_number)

    def bag_to_kitti(self, seq_name, robot_number):
        print(f"    * robot: robot{robot_number}")

        # Create KITTI-style directory for run
        kitti_paths_dict = self.create_run_kitti_directory(seq_name=seq_name, robot_number=robot_number)
        kitti_directory_path = kitti_paths_dict['directory_path']
        print(f"        - Created KITTI-style directory: {kitti_directory_path}")

        # Init bagparser object
        robot_name = f"robot{robot_number:01d}"
        bag_parser = BagParser(log_paths_dict=kitti_paths_dict, robot_name=robot_name)

        # Parse rosbag data
        bag_path = os.path.join(self.lidar2osm_config_dict["main_bag_directory_path"], seq_name, "rosbags/raw", f"{robot_name}_{seq_name}_raw.bag")
        print(f"        - Parsing rosbag: {bag_path}")
        bag_parser.read_bag(rosbag_path=bag_path)

        # Save data
        bag_parser.write_data_to_files()

        # # Compress kitti-style directory and delete uncompressed
        # print(f"        - Compressing and removing uncompressed directory: {kitti_directory_path}")
        # compressed_path = shutil.make_archive(kitti_directory_path, 'zip', kitti_directory_path)
        # shutil.rmtree(kitti_directory_path)
        # print(f"        - Run {run_name} processed and compressed to {compressed_path}")

    def create_run_kitti_directory(self, seq_name, robot_number):
        """_Create directory to store binarized bag."""
        # Make sure base directory for run exists or create it
        root_binarized_dir = self.lidar2osm_config_dict["main_output_directory_path"]
        directory_path = os.path.join(root_binarized_dir, seq_name, "binarized", f"robot{robot_number:01d}")

        # Dictionary to hold all necessary subdirectory paths
        run_dir_paths_dict = {
            'directory_path': directory_path,
            'poses_path': os.path.join(directory_path, "poses"),
            'lidar_path': os.path.join(directory_path, "lidar"),
            'lidar_pc_bin_path': os.path.join(directory_path, "lidar", "pointclouds"),
            'gps_path': os.path.join(directory_path, "gps"),
            'imu_path': os.path.join(directory_path, "imu"),
            # 'camera_path': os.path.join(directory_path, "camera"),
            # 'camera_rgb_path': os.path.join(directory_path, "camera", "images", "rgb"),
            # 'camera_depth_path': os.path.join(directory_path, "camera", "images", "depth")
        }

        # Create all directories based on the paths in the dictionary
        for path in run_dir_paths_dict.values():
            os.makedirs(path, exist_ok=True)

        return run_dir_paths_dict