import os

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import osmnx as ox
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm

from utils.convert_oxts_pose import *


def load_bin_file(file_path, shape, dtype=np.float32):
    """
    Load binary file and reshape it into the desired shape.
    """
    return np.fromfile(file_path, dtype=dtype).reshape(shape)


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


def get_osm_buildings_points(osm_file_path):
    # Filter features for buildings and sidewalks
    buildings = ox.features_from_xml(osm_file_path, tags={"building": True})
    osm_building_list = []
    # Process Buildings as LineSets
    for _, building in buildings.iterrows():
        if building.geometry.geom_type == "Polygon":
            exterior_coords = building.geometry.exterior.coords
            for i in range(len(exterior_coords) - 1):
                start_point = [exterior_coords[i][0], exterior_coords[i][1], 0]
                end_point = [
                    exterior_coords[i + 1][0],
                    exterior_coords[i + 1][1],
                    0,
                ]
                osm_building_list.append(start_point)
                osm_building_list.append(end_point)
    osm_building_points = np.array(osm_building_list)
    return osm_building_points


def get_osm_road_points(osm_file_path):
    # For below tags, see: https://wiki.openstreetmap.org/wiki/Key:highway#Roads
    # tags_OG = {'highway': ['residential', 'tertiary']}  # This will fetch tertiary and residential roads
    tags = {
        "highway": [
            "motorway",
            "trunk",
            "primary",
            "secondary",
            "tertiary",
            "unclassified",
            "residential",
            "motorway_link",
            "trunk_link",
            "primary_link",
            "secondary_link",
            "tertiary_link",
            "living_street",
            "service",
            "pedestrian",
            "road",
            "cycleway",
            "foot",
            "footway",
            "path",
            "service",
        ]
    }

    # Fetch roads using defined tags
    roads = ox.features_from_xml(osm_file_path, tags=tags)

    # Process Roads as LineSets with width
    osm_road_list = []
    for _, road in roads.iterrows():
        if road.geometry.geom_type == "LineString":
            coords = np.array(road.geometry.xy).T
            road_center = [
                np.mean(np.array(coords)[:, 0]),
                np.mean(np.array(coords)[:, 1]),
            ]
            for i in range(len(coords) - 1):
                start_point = [
                    coords[i][0],
                    coords[i][1],
                    0,
                ]  # Assuming roads are at ground level (z=0)
                end_point = [coords[i + 1][0], coords[i + 1][1], 0]
                osm_road_list.append(start_point)
                osm_road_list.append(end_point)
    osm_road_points = np.array(osm_road_list)
    return osm_road_points


def convert_polyline_points_to_o3d(polyline_points, rgb_color):
    polyline_pcd = o3d.geometry.LineSet()
    if len(polyline_points) > 0:
        polyline_lines_idx = [[i, i + 1] for i in range(0, len(polyline_points) - 1, 2)]
        polyline_pcd.points = o3d.utility.Vector3dVector(polyline_points)
        polyline_pcd.lines = o3d.utility.Vector2iVector(polyline_lines_idx)
        polyline_pcd.paint_uniform_color(rgb_color)
    return polyline_pcd


def transform_points_lat_lon(point_cloud, pose_matrix):
    pc = transform_point_cloud(point_cloud, pose_matrix)
    pc_reshaped = np.array([np.eye(4) for _ in range(pc.shape[0])])
    pc_reshaped[:, 0:3, 3] = pc[:, :3]
    pc_lla = np.asarray(postprocessPoses(pc_reshaped))
    pc_lla = np.asarray(convert_pointcloud_to_latlon(pc_lla))[:, :3]
    pc_lla[:, 2] *= 0.00001
    return pc_lla


# Directory paths
poses_directory = "./data/ouster/poses/"
pointcloud_directory = "./data/ouster/data/"

# List of pose files sorted by timestamp
pose_files = sorted(os.listdir(poses_directory))[::200]  # Get every 10th pose file

# Accumulate all transformed point clouds
accumulated_points = []
all_intensities = []
all_reflectivities = []

# Initialize list for frames
frames = []
for pose_file in tqdm(pose_files, desc="Processing Pose Files", unit="file"):
    # Load the pose matrix
    pose_matrix_path = os.path.join(poses_directory, pose_file)
    pose_matrix = load_bin_file(pose_matrix_path, shape=(4, 4))

    # Create a coordinate frame for each pose
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=1.0, origin=pose_matrix[:3, 3]
    )
    frame.rotate(pose_matrix[:3, :3], center=pose_matrix[:3, 3])
    frames.append(frame)

    # Find the corresponding point cloud file
    timestamp = pose_file.replace(".bin", "")
    pointcloud_file = f"{timestamp}.bin"
    pointcloud_path = os.path.join(pointcloud_directory, pointcloud_file)

    # Load the point cloud
    if os.path.exists(pointcloud_path):
        # Assuming point cloud has columns: x, y, z, intensity, reflectivity
        point_cloud = load_bin_file(pointcloud_path, shape=(-1, 5))
        point_cloud_xyz = point_cloud[:, :3]
        point_cloud_intensity = point_cloud[:, 3]
        point_cloud_reflectivity = point_cloud[:, 4]

        all_intensities.extend(point_cloud_intensity)
        all_reflectivities.extend(point_cloud_reflectivity)

        # Transform the point cloud using the pose matrix
        transformed_points = transform_points_lat_lon(point_cloud_xyz, pose_matrix)

        # Accumulate transformed points
        accumulated_points.append(transformed_points)
    else:
        print(f"Point cloud file {pointcloud_file} not found.")

# Combine all accumulated points into a single array
if accumulated_points:
    all_points = np.vstack(accumulated_points)
    all_intensities = np.hstack(all_intensities)
    all_reflectivities = np.hstack(all_reflectivities)
    all_z_values = all_points[:, 2]

    # Visualize using Open3D
    point_cloud_o3d = o3d.geometry.PointCloud()
    point_cloud_o3d.points = o3d.utility.Vector3dVector(all_points)

    # Normalize intensity and reflectivity using mean and standard deviation
    intensity_mean = np.mean(all_intensities)
    intensity_std = np.std(all_intensities)
    reflectivity_mean = np.mean(all_reflectivities)
    reflectivity_std = np.std(all_reflectivities)
    z_mean = np.mean(all_z_values)
    z_std = np.std(all_z_values)

    # Normalize using the standard deviation
    intensity_normalized = (all_intensities - intensity_mean) / intensity_std
    reflectivity_normalized = (
        all_reflectivities - reflectivity_mean
    ) / reflectivity_std
    z_normalized = (all_z_values - z_mean) / z_std

    # Clip to range [0, 1] for visualization purposes
    z_normalized = np.clip(z_normalized, 0, 1)
    intensity_normalized = np.clip(intensity_normalized, 0, 1)
    reflectivity_normalized = np.clip(reflectivity_normalized, 0, 1)

    # Combine intensity and reflectivity for RGB mapping
    # Option 1: Average
    combined_attribute = (
        intensity_normalized + reflectivity_normalized + z_normalized
    ) / 3

    # Option 2: Different channels
    # combined_attribute = np.stack((intensity_normalized, reflectivity_normalized, reflectivity_normalized), axis=1)

    # Use a colormap to map combined attributes to RGB colors
    colormap = plt.get_cmap("viridis")  # Choose a colormap: 'viridis', 'jet', etc.
    colors = colormap(combined_attribute)[:, :3]  # Ignore the alpha channel
    point_cloud_o3d.colors = o3d.utility.Vector3dVector(colors)

    # Downsample the point cloud using voxel downsampling
    voxel_size = 0.2  # Adjust this value to control downsampling resolution
    # downsampled_point_cloud = point_cloud_o3d.voxel_down_sample(voxel_size=voxel_size)

    osm_file_path = "./UCB_MAIN.osm"
    osm_building_points = get_osm_buildings_points(osm_file_path)
    osm_road_points = get_osm_road_points(osm_file_path)
    osm_road_pcd = convert_polyline_points_to_o3d(osm_road_points, [1, 0, 0])
    osm_building_pcd = convert_polyline_points_to_o3d(osm_building_points, [0, 0, 1])

    # Add coordinate frames to the scene
    geometry_list = [point_cloud_o3d, osm_building_pcd]

    # Visualize using Open3D
    o3d.visualization.draw_geometries(
        geometry_list,
        window_name="Downsampled Accumulated Point Cloud with Pose Frames",
        width=800,
        height=600,
    )
else:
    print("No point clouds found to accumulate.")
