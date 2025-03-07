#!/usr/bin/env python3
import yaml
from tqdm import tqdm
from dataset_to_bin import DatasetToBin

def parse_config(config_path):
    """_Parser for ColoradarPlus-to-kitti Configuration file._

    Args:
        config_path (_str_): _Path to the YAML configuration file._

    Returns:
       config_values (_dict_): _Python dict containing values taken from YAML configuration file._
    """
    def load_config(path):
        with open(path, "r") as file:
            return yaml.safe_load(file)
        
    config = load_config(config_path)

    # Configuration as a dict
    dataset_config_dict = {
        "main_bag_directory_path": config.get("main_bag_directory_path", []),
        "main_output_directory_path": config.get("main_output_directory_path", []),
        "generate_sequence_run_stats": (config.get("generate_sequence_run_stats", False)),
        "display_rosbag_data": (config.get("display_rosbag_data", False)),
        "sequences": (
            config["sequences"]
        ),
    }
    return dataset_config_dict


def main():
    # Convert values from YAML config to dict for easier handling
    lidar2osm_config_dict = parse_config("config/robot1_config.yaml")

    # Initialize bag_parser object
    dataset2bin = DatasetToBin(lidar2osm_config_dict)

    # Convert specified robots in sequences to binarized dataset
    dataset2bin.convert()

if __name__ == "__main__":
    main()