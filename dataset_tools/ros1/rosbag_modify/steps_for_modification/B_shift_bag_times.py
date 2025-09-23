"""
    multi_robot_bag.py
"""
import rosbag
import rospy
import os

desired_topics = [
    "/camera/color/camera_info",
    "/camera/color/image_raw",
    "/camera/color/image_raw/theora",
    "/camera/color/image_raw/theora/parameter_descriptions",
    "/camera/color/image_raw/theora/parameter_updates",
    "/camera/color/metadata",
    "/camera/depth/camera_info",
    "/camera/depth/color/points",
    "/camera/depth/image_rect_raw",
    "/camera/depth/metadata",
    "/camera/extrinsics/depth_to_color",
    "/camera/pointcloud/parameter_descriptions",
    "/camera/pointcloud/parameter_updates",
    "/camera/realsense2_camera_manager/bond",
    "/camera/rgb_camera/auto_exposure_roi/parameter_descriptions",
    "/camera/rgb_camera/auto_exposure_roi/parameter_updates",
    "/camera/rgb_camera/parameter_descriptions",
    "/camera/rgb_camera/parameter_updates",
    "/camera/stereo_module/auto_exposure_roi/parameter_descriptions",
    "/camera/stereo_module/auto_exposure_roi/parameter_updates",
    "/camera/stereo_module/parameter_descriptions",
    "/camera/stereo_module/parameter_updates",
    "/diagnostics",
    "/ekf/dual_antenna_heading",
    "/ekf/imu/data",
    "/ekf/llh_position",
    "/ekf/odometry_earth",
    "/ekf/odometry_map",
    "/ekf/status",
    "/ekf/velocity",
    "/gnss_1/llh_position",
    "/gnss_1/odometry_earth",
    "/gnss_1/time",
    "/gnss_1/velocity",
    "/gnss_1/velocity_ecef",
    "/gnss_2/llh_position",
    "/gnss_2/odometry_earth",
    "/gnss_2/time",
    "/gnss_2/velocity",
    "/gnss_2/velocity_ecef",
    "/imu/data",
    "/imu/mag",
    "/imu/pressure",
    "/mip/ekf/aiding_measurement_summary",
    "/mip/ekf/gnss_dual_antenna_status",
    "/mip/ekf/gnss_position_aiding_status",
    "/mip/ekf/multi_antenna_offset_correction",
    "/mip/ekf/status",
    "/mip/gnss_1/fix_info",
    "/mip/gnss_1/rf_error_detection",
    "/mip/gnss_1/sbas_info",
    "/mip/gnss_2/fix_info",
    "/mip/gnss_2/rf_error_detection",
    "/mip/gnss_2/sbas_info",
    "/mip/gnss_corrections/rtk_corrections_status",
    "/mip/sensor/overrange_status",
    "/ouster/imu",
    # "/ouster/imu_packets",
    # "/ouster/lidar_packets",
    "/ouster/metadata",
    # "/ouster/nearir_image",
    # "/ouster/os_nodelet_mgr/bond",
    "/ouster/points",
    # "/ouster/range_image",
    # "/ouster/reflec_image",
    # "/ouster/scan",
    # "/ouster/signal_image"
]


def shift_bag_start_time(target_time_bag_path, source_time_bag_path, adjusted_time_bag_path):
    """
    Shifts the start time of a ROS bag file to a specific Unix timestamp.

    Parameters:
        input_bag_path (str): Path to the input bag file.
        output_bag_path (str): Path to the output bag file.
        new_start_time (float): The desired start time in Unix timestamp.
    """
    with rosbag.Bag(target_time_bag_path, 'r') as inbag:
        # Get the current start time of the bag
        new_start_time = inbag.get_start_time()

    with rosbag.Bag(source_time_bag_path, 'r') as inbag:
        # Get the current start time of the bag
        current_start_time = inbag.get_start_time()

        # Calculate the shift needed
        time_shift = new_start_time - current_start_time

    print(f"\nnew_start_time: {rospy.Duration(new_start_time)}, current_start_time: {rospy.Duration(current_start_time)}")
    print(f"time_shift: {rospy.Duration(time_shift)}\n")

    with rosbag.Bag(adjusted_time_bag_path, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(source_time_bag_path).read_messages(desired_topics):
            # if time_shift > 0:
            # Shift the timestamp of the message
            new_time = t + rospy.Duration(time_shift)

            # Adjust the message header timestamps
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                if msg.header.stamp.to_sec() > 0:
                    msg.header.stamp += rospy.Duration(time_shift)

            # Adjust transform timestamps
            if hasattr(msg, 'transforms'):
                for transform in msg.transforms:
                    if transform.header.stamp.to_sec() > 0:
                        transform.header.stamp += rospy.Duration(time_shift)
            
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                print(f"\ntopic: {topic}, old_time: {t}, new_time: {new_time}, msg.header.stamp: {msg.header.stamp}")

            # Write the message with the new timestamp to the output bag
            outbag.write(topic, msg, new_time)


def shift_start_times(target_time_bag_path, inbag_paths, outbag_paths):
    for paths in zip(inbag_paths, outbag_paths):
        print(f"\nShifting start time of {paths[0]}...")
        shift_bag_start_time(target_time_bag_path, paths[0], paths[1])
        print(f"Removing original bag...")
        os.remove(paths[0])


def main():
    env = "main_campus"
    root_input_dir = f"/root/datasets/{env}/filtered"
    root_output_dir = f"/root/datasets/{env}/shifted"
    robot_prefixes = ['robot1', 'robot2', 'robot3', 'robot4']

    target_time_bag_path = f"{root_input_dir}/{robot_prefixes[0]}_{env}_filtered.bag"
    inbag_paths = [f"{root_input_dir}/{pre}_{env}_filtered.bag" for pre in robot_prefixes[1:]]
    outbag_paths =  [f"{root_output_dir}/{pre}_{env}_shifted.bag" for pre in robot_prefixes[1:]]
    
    shift_start_times(target_time_bag_path, inbag_paths, outbag_paths)


if __name__ == '__main__':
    main()