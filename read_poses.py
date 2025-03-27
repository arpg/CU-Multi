import os
from decimal import Decimal
import numpy as np
import open3d as o3d
from scipy.spatial import KDTree
# OVERLAP OVER TOTAL
class Robot:
    def __init__(self, env, robot_number):
        self.env = env
        self.number = robot_number
        self.name = f"robot{robot_number}"
        # print(f"Initializing {self.name}...")
        self.ts2pose_map = {}  
        self.poses = []
        self.timestamps = []
        self.overlap_indices = self.set_overlap_indices()
        self.set_data()
    
    def set_overlap_indices(self):
        return set()

    def set_data(self):
        root_dir = os.path.join(self.env, self.name)
        # print(f"root_dir: {root_dir}")
        timestamps_path = os.path.join(root_dir, "timestamps.txt")
        poses_path = os.path.join(root_dir, "poses_world.txt")

        if not os.path.exists(timestamps_path) or not os.path.exists(poses_path):
            # print(f"Warning: Missing data files for {self.name}")
            return

        with open(timestamps_path, "r") as f:
            self.timestamps = [Decimal(line.strip()) for line in f]
            # print(f"Timestamps len: {len(self.timestamps)}")
        with open(poses_path, "r") as f:
            self.poses = [tuple(map(float, line.strip().split())) for line in f]
            # print(f"poses len: {len(self.poses)}")
        self.ts2pose_map = dict(zip(self.timestamps, self.poses))
        # print()

    def compute_path_length(self):
        """ Computes the total trajectory length of the robot. """
        # print(f"[compute_path_length]")
        if not self.poses:
            return 0.0  

        poses = np.array(self.poses)[:, :3]  # Extract XYZ
        distances = np.linalg.norm(np.diff(poses, axis=0), axis=1)
        return np.sum(distances)

    def compute_loop_closures(self, robot_poses, distance_threshold, min_time_gap):
        """
        Detects loop closures (self-overlapping paths) and computes overlap length of Source robot to Target robot
        
        :param distance_threshold: Max Euclidean distance to consider as overlap (meters)
        :param min_time_gap: Minimum time gap to avoid immediate neighbors
        :return: Total overlapping distance (meters)
        """
        if len(robot_poses) < 2:
            return 0.0

        target_poses = np.array(robot_poses)[:, :2]  
        source_poses = np.array(self.poses)[:,:2]
        tree = KDTree(target_poses)  

        total_overlap_distance = 0.0
        accumulating = False  

        for i in range(len(source_poses) - 1):
            # Find past points close to the current position
            neighbors = tree.query_ball_point(source_poses[i], distance_threshold)
            # print(neighbors)
            # Exclude recent past points
            valid_neighbors = [n for n in neighbors if abs(n - i) >= min_time_gap]

            if valid_neighbors:
                self.overlap_indices.add(i)  
                if not accumulating:
                    accumulating = True  
            else:
                if accumulating:
                    accumulating = False  

            if accumulating:
                step_distance = np.linalg.norm(source_poses[i] - source_poses[i + 1])
                total_overlap_distance += step_distance
        
        # for idx in self.overlap_indices:
        #     print(idx)
        return total_overlap_distance


def plot_trajectories(robot_list, point_size=5):
    ''' Plots robot trajectories as a point cloud with loop closures highlighted. '''
    # print("[plot_trajectories]")
    colors = {
        1: [0, 0.6, 0],  # Green
        2: [0, 0, 1],  # Purple
        3: [1, 0.5, 0],  # Orange
        4: [0, 0, 1]   # Blue
    }
    overlap_color = [1, 0, 0]  # Red for overlapping points
    point_clouds = []
    for robot in robot_list:
        # print(robot)
        if not robot.poses:
            print(f"No robot poses: {len(robot.poses)}")
            continue
        poses = np.array(robot.poses)[:, :3].astype(np.float64)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(poses))
        # Assign colors: Red for overlap, normal for rest
        point_colors = []
        for i in range(len(poses)):
            if i in robot.overlap_indices:
                point_colors.append(overlap_color)
            else:
                point_colors.append(colors[robot.number])

        # print(f"Point Colors len: {np.array(point_colors).shape}")
        pcd.colors = o3d.utility.Vector3dVector(np.array(point_colors))
        # print(f"PCD: {pcd}")
        point_clouds.append(pcd)
        
    # Create visualization window
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    for i, pcd in enumerate(point_clouds):
        vis.add_geometry(pcd)

    # Adjust point sizes
    opt = vis.get_render_option()
    opt.point_size = point_size  
    vis.run()
    vis.destroy_window()


# Initialize robots

robot_nums = [1, 2, 3, 4]
distance_threshold = 1
min_time_gap = 600
# print(f"[INTRA ROBOT] Distance Threshold: {distance_threshold}m, Min Point In Between: {min_time_gap}")
# for env_name in ["main_campus","kittredge_loop"]:
#     print(f"        =========={env_name}==========")
#     robot_list = [Robot(env_name, robot_num) for robot_num in robot_nums]
#     for robot in robot_list:
#         path_length = robot.compute_path_length()
#         overlap_length = robot.compute_loop_closures(robot.poses, distance_threshold, min_time_gap)
#         print(f"        {robot.name}: Overlap {overlap_length:.4f} / Path length {path_length:.4f} = {overlap_length/path_length:.4}")
#     plot_trajectories(robot_list, point_size=3)
    # percentages = [0.3472, 0.3019, 0.3458, 0.2872, 0.1603, 0.2336, 0.2966, 0.1804]
print(f"Distance Threshold: {distance_threshold}m, Min Point In Between: {min_time_gap}")
matrices_pct = {}
matrices_dis = {}
for env_name in ["main_campus","kittredge_loop"]:
    matrices_pct[env_name] = []
    matrices_dis[env_name] = []
    print(f"=========={env_name}==========")
    robot_list = [Robot(env_name, robot_num) for robot_num in robot_nums]
    for robot_source in robot_list:
        row_pct = []
        row_dis = []
        for robot_target in robot_list:
            # reset the overlap indices
            robot_source.overlap_indices = robot_source.set_overlap_indices()
            path_length = robot_source.compute_path_length()
            if robot_target != robot_source:
                min_time_gap = 0
            else:
                min_time_gap = 600
            overlap_length = robot_source.compute_loop_closures(robot_target.poses, distance_threshold, min_time_gap)
            overlap_percent = overlap_length/path_length
            print(f"Min Gap: {min_time_gap}, Source: [{robot_source.name}], Target [{robot_target.name}]: Overlap {overlap_length:.4f} / Path length {path_length:.4f} = {overlap_percent:.4}")
            row_pct.append(overlap_percent)
            row_dis.append(overlap_length)
            plot_trajectories([robot_source, robot_target])
        matrices_pct[env_name].append(row_pct)
        matrices_dis[env_name].append(row_dis)
        # plot_trajectories(robot_list, point_size=3)


print()
# print(f"{}")
for env_name in matrices_pct.keys():
    print(f"{env_name}")
    for row in matrices_pct[env_name]:
        print(" | ".join(f"{num:.2f}" for num in list(row)))

for env_name in matrices_dis.keys():
    print(f"{env_name}")
    for row in matrices_dis[env_name]:
        print(" | ".join(f"{num:.2f}" for num in list(row)))
