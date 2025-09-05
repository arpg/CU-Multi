import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosbag2_py import StorageFilter
import matplotlib.pyplot as plt

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

def read_first_pointcloud_and_print_times(bag_path, topic_name):
    # Setup rosbag2 reader
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # List topics (optional)
    for t in reader.get_all_topics_and_types():
        print(t.name)

    # Filter only the desired topic
    reader.set_filter(StorageFilter(topics=[topic_name]))
    print(f"Reading first PointCloud2 message from topic: {topic_name}")

    while reader.has_next():
        (topic, data, bag_time_ns) = reader.read_next()
        if topic != topic_name:
            continue

        msg = deserialize_message(data, PointCloud2)
        print(f"Found point cloud with timestamp: {bag_time_ns} (bag time in nanoseconds)")

        # Identify time field
        field_names = [f.name for f in msg.fields]
        if 't' in field_names:
            t_index = field_names.index('t')
        elif 'time' in field_names:
            t_index = field_names.index('time')
        else:
            print("No 't' or 'time' field present in the PointCloud2; cannot extract per-point times.")
            break

        # Collect per-point time offsets (ns)
        point_times = []
        for pt in pc2.read_points(msg, field_names=None, skip_nans=True):
            point_times.append(pt[t_index])

        if not point_times:
            print("No point times extracted.")
            break

        # Sort and report
        point_times_sorted = sorted(point_times)
        t_min = point_times_sorted[0]
        t_max = point_times_sorted[-1]
        duration_ns = int(t_max - t_min)
        duration_s = duration_ns * 1e-9 if duration_ns > 0 else 0.0
        est_hz = (1.0 / duration_s) if duration_s > 0 else float('inf')

        print(f"Per-point time is an offset from scan start (nanoseconds).")
        print(f"min(t): {t_min} ns")
        print(f"max(t): {t_max} ns")
        print(f"scan duration: {duration_ns} ns  (~{duration_s:.6f} s)")
        print(f"estimated scan rate: {est_hz:.3f} Hz")

        # Optional: absolute per-point times = bag timestamp + offset
        # absolute_times_ns = [bag_time_ns + int(x) for x in point_times_sorted]

        # Plot index vs sorted time offset
        plt.figure(figsize=(10, 4))
        plt.plot(point_times_sorted, '.', markersize=2)
        plt.title("Per-point time offsets within scan (sorted)")
        plt.xlabel("Point index (sorted by time)")
        plt.ylabel("Time offset from scan start (ns)")
        plt.grid(True)
        plt.show()
        break

        
# def read_first_pointcloud_and_print_times(bag_path, topic_name):
#     # Setup rosbag2 reader
#     storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
#     converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
#     reader = SequentialReader()
#     reader.open(storage_options, converter_options)

#     # Get all topics and types
#     topics = reader.get_all_topics_and_types()
    
#     for t in topics:
#         print(t.name)

#     topic_type_dict = {t.name: t.type for t in topics}

#     # Filter only the desired topic
#     reader.set_filter(StorageFilter(topics=[topic_name]))

#     print(f"Reading first PointCloud2 message from topic: {topic_name}")

#     # Read messages
#     while reader.has_next():
#         (topic, data, t) = reader.read_next()
#         if topic == topic_name:
#             msg = deserialize_message(data, PointCloud2)

#             print(f"Found point cloud with timestamp: {t} (bag time in nanoseconds)")

#             # Extract timestamps
#             point_generator = pc2.read_points(msg, field_names=None, skip_nans=True)
#             field_names = [f.name for f in msg.fields]

#             point_times = []
#             if 't' in field_names:
#                 t_index = field_names.index('t')
#             elif 'time' in field_names:
#                 t_index = field_names.index('time')
#             else:
#                 print("No time or t field in point cloud. Cannot extract individual point times.")
#                 break

#             for point in point_generator:
#                 point_times.append(point[t_index])

#             if point_times:
#                 point_times_sorted = sorted(point_times)

#                 print(f"First point time: {point_times_sorted[0]}")
#                 print(f"Last point time: {point_times_sorted[-1]}")

#                 # Plot
#                 plt.figure(figsize=(10, 5))
#                 plt.plot(point_times_sorted, '.', markersize=2)
#                 plt.title("Point Times (sorted)")
#                 plt.xlabel("Point index")
#                 plt.ylabel("Time")
#                 plt.grid(True)
#                 plt.show()

#             else:
#                 print("Could not extract point times from fields.")
#             break

# def read_first_pointcloud_and_print_times(bag_path, topic_name):
#     # Setup rosbag2 reader
#     storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
#     converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
#     reader = SequentialReader()
#     reader.open(storage_options, converter_options)

#     # Get all topics and types
#     topics = reader.get_all_topics_and_types()
    
#     for t in topics:
#         print(t.name)

#     topic_type_dict = {t.name: t.type for t in topics}

#     # Filter only the desired topic
#     # reader.set_filter({'topics': [topic_name]})
#     reader.set_filter(StorageFilter(topics=[topic_name]))

#     print(f"Reading first PointCloud2 message from topic: {topic_name}")

#     # Read messages
#     while reader.has_next():
#         (topic, data, t) = reader.read_next()
#         if topic == topic_name:
#             msg = deserialize_message(data, PointCloud2)

#             print(f"Found point cloud with timestamp: {t} (bag time in nanoseconds)")

#             # Try to extract timestamps from the point cloud
#             point_generator = pc2.read_points(msg, field_names=None, skip_nans=True)

#             point_times = []

#             for point in point_generator:
#                 # Look for 't' or 'time' field in the point tuple
#                 # Assuming fields are (x, y, z, t) or similar
#                 if 't' in [f.name for f in msg.fields]:
#                     # Find index of 't' field
#                     t_index = [f.name for f in msg.fields].index('t')
#                     point_times.append(point[t_index])
#                 elif 'time' in [f.name for f in msg.fields]:
#                     t_index = [f.name for f in msg.fields].index('time')
#                     point_times.append(point[t_index])
#                 else:
#                     print("No time or t field in point cloud. Cannot extract individual point times.")
#                     break

#             if point_times:
#                 print(f"First point time: {point_times[0]}")
#                 print(f"Last point time: {point_times[-1]}")
#             else:
#                 print("Could not extract point times from fields.")

#             break  # Only process the first message

if __name__ == "__main__":
    import sys
    # if len(sys.argv) != 3:
    #     print("Usage: python read_pointcloud_times.py <path_to_rosbag2> <topic_name>")
    #     sys.exit(1)

    rclpy.init()
    bag_path = "/media/donceykong/doncey_ssd_03/CU_MULTI/main_campus/robot1/robot1_main_campus_lidar" #sys.argv[1]
    topic_name = "robot1/ouster/points" #sys.argv[2]

    read_first_pointcloud_and_print_times(bag_path, topic_name)
    rclpy.shutdown()

