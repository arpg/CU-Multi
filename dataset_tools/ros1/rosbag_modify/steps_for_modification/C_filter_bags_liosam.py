"""
    multi_robot_bag.py
"""
import rosbag
import rospy
import os
from tqdm import tqdm

camera_topics = [
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
]

lidar_imu_gps_topics = [
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

topics_dict = {
    # 'camera': camera_topics,
    'lidar_imu_gps': lidar_imu_gps_topics
}


def filter_bag_into_topics(inbag_path, filtered_bag_path, topics):
    inbag = rosbag.Bag(inbag_path)
    start_time = inbag.get_start_time()
    end_time = inbag.get_end_time()
    duration = end_time - start_time

    with rosbag.Bag(filtered_bag_path, 'w') as outbag:
        with tqdm(total=duration, desc=f"Filtering for {topics}", unit="sec") as pbar:
            for topic, msg, t in inbag.read_messages(topics=topics_dict[topics]):
                outbag.write(topic, msg, t)
                current_time = t.to_sec()
                pbar.n = current_time - start_time
                pbar.refresh()  # Manually refresh since we set .n directly

    inbag.close()


def main():
    env = "main_campus"
    root_dir = f"/root/datasets/{env}"
    robot_prefixes = ['robot1', 'robot2', 'robot3', 'robot4']

    for robot in robot_prefixes:
        print("\n")
        print(f"Filtering {robot} into topics-based bags.")
        inbag_path = f"{root_dir}/shifted/{robot}_{env}_shifted.bag"

        for topics in topics_dict.keys():
            outbag_path = f"{root_dir}/split/{robot}_{env}_{topics}.bag"
            filter_bag_into_topics(inbag_path, outbag_path, topics)
        print("\n")


if __name__ == '__main__':
    main()