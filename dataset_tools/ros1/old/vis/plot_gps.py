import numpy as np
import open3d as o3d
import osmnx as ox


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


# Function to read GPS data from a file
def read_gps_data(file_path):
    points = []
    with open(file_path, "r") as file:
        for line in file:
            lat, lon, alt = map(float, line.strip().split())
            points.append([lon, lat, alt])
    return np.array(points)


# Convert lat/lon to a local coordinate system (e.g., ECEF or a local tangent plane)
def latlon_to_ecef(lat, lon, alt):
    # Constants for WGS84
    a = 6378137.0  # Earth's semi-major axis in meters
    f = 1 / 298.257223563  # Flattening
    e2 = f * (2 - f)  # Square of eccentricity

    lat = np.radians(lat)
    lon = np.radians(lon)

    N = a / np.sqrt(1 - e2 * np.sin(lat) ** 2)
    X = (N + alt) * np.cos(lat) * np.cos(lon)
    Y = (N + alt) * np.cos(lat) * np.sin(lon)
    Z = (N * (1 - e2) + alt) * np.sin(lat)

    return X, Y, Z


def convert_to_local_coordinates(points):
    ecef_points = []
    for point in points:
        ecef_points.append(latlon_to_ecef(point[0], point[1], point[2]))
    return np.array(ecef_points)


# Function to display points using Open3D
def display(
    osm_road_points, osm_building_points, gps_1_points, gps_2_points, gps_ekf_points
):
    osm_road_pcd = convert_polyline_points_to_o3d(osm_road_points, [1, 0, 0])
    osm_building_pcd = convert_polyline_points_to_o3d(osm_building_points, [0, 0, 1])

    gps_1_point_cloud = o3d.geometry.PointCloud()
    gps_1_point_cloud.points = o3d.utility.Vector3dVector(gps_1_points[:10000])
    gps_1_point_cloud.paint_uniform_color([1, 0, 0])

    gps_2_point_cloud = o3d.geometry.PointCloud()
    gps_2_point_cloud.points = o3d.utility.Vector3dVector(gps_2_points[:10000])
    gps_2_point_cloud.paint_uniform_color([0, 1, 0])

    gps_ekf_point_cloud = o3d.geometry.PointCloud()
    gps_ekf_point_cloud.points = o3d.utility.Vector3dVector(gps_ekf_points[:10000])
    gps_ekf_point_cloud.paint_uniform_color([0, 1, 1])

    o3d.visualization.draw_geometries(
        [
            osm_road_pcd,
            osm_building_pcd,
            gps_1_point_cloud,
            gps_2_point_cloud,
            gps_ekf_point_cloud,
        ]
    )


# Main execution
gps_1_file_path = "./data/gps/gnss_1_data.txt"
gps_1_points = read_gps_data(gps_1_file_path)
gps_1_points[:, 2] = 0

gps_2_file_path = "./data/gps/gnss_2_data.txt"
gps_2_points = read_gps_data(gps_2_file_path)
gps_2_points[:, 2] = 0

gps_ekf_file_path = "./data/gps/gnss_ekf_data.txt"
gps_ekf_points = read_gps_data(gps_ekf_file_path)
gps_ekf_points[:, 2] = 0
mask = ~np.all(gps_ekf_points == 0, axis=1)  # mask for zero-only rows
filtered_gps_ekf_points = gps_ekf_points[mask]

print(
    f"ave 100 first ekf position: {np.mean(filtered_gps_ekf_points[:100, 0]), np.mean(filtered_gps_ekf_points[:100, 1]), np.mean(filtered_gps_ekf_points[:100, 2])}"
)
# gps_average = (gps_1_points + gps_2_points) / 2
# ecef_points = convert_to_local_coordinates(gps_points)

osm_file_path = "./UCB_MAIN.osm"
osm_building_points = get_osm_buildings_points(osm_file_path)
osm_road_points = get_osm_road_points(osm_file_path)

display(
    osm_road_points,
    osm_building_points,
    gps_1_points,
    gps_2_points,
    filtered_gps_ekf_points,
)
