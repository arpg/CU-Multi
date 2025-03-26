import os
from decimal import Decimal
import numpy as np
import open3d as o3d
from scipy.spatial import KDTree

class Robot:
    def __init__(self, env, robot_number):
        self.env = env
        self.number = robot_number
        self.name = f"robot{robot_number}"
        print(f"Initializing {self.name}...")
        self.ts2pose_map = {}  
        self.poses = []
        self.timestamps = []
        self.overlap_indices = set()
        self.set_data()

    def set_data(self):
        root_dir = os.path.join(self.env, self.name)
        timestamps_path = os.path.join(root_dir, "timestamps.txt")
        poses_path = os.path.join(root_dir, "poses_world.txt")

        if not os.path.exists(timestamps_path) or not os.path.exists(poses_path):
            print(f"Warning: Missing data files for {self.name}")
            return

        with open(timestamps_path, "r") as f:
            self.timestamps = [Decimal(line.strip()) for line in f]

        with open(poses_path, "r") as f:
            self.poses = [tuple(map(float, line.strip().split())) for line in f]

        self.ts2pose_map = dict(zip(self.timestamps, self.poses))

    def compute_path_length(self):
        """ Computes the total trajectory length of the robot. """
        if not self.poses:
            return 0.0  

        poses = np.array(self.poses)[:, :3]  # Extract XYZ
        distances = np.linalg.norm(np.diff(poses, axis=0), axis=1)
        return np.sum(distances)

    def compute_loop_closures(self, distance_threshold=0.5, min_time_gap=5):
        """
        Detects loop closures (self-overlapping paths) and computes overlap length.
        
        :param distance_threshold: Max Euclidean distance to consider as overlap (meters)
        :param min_time_gap: Minimum time gap to avoid immediate neighbors
        :return: Total overlapping distance (meters)
        """
        if len(self.poses) < 2:
            return 0.0

        poses = np.array(self.poses)[:, :3]  
        tree = KDTree(poses)  

        self.overlap_indices = set()
        total_overlap_distance = 0.0
        accumulating = False  

        for i in range(len(poses) - 1):
            # Find past points close to the current position
            neighbors = tree.query_ball_point(poses[i], distance_threshold)

            # Exclude recent past points
            valid_neighbors = [n for n in neighbors if abs(n - i) > min_time_gap]

            if valid_neighbors:
                self.overlap_indices.add(i)  
                if not accumulating:
                    accumulating = True  
            else:
                if accumulating:
                    accumulating = False  

            if accumulating:
                step_distance = np.linalg.norm(poses[i] - poses[i + 1])
                total_overlap_distance += step_distance
        
        # for idx in self.overlap_indices:
        #     print(idx)
        return total_overlap_distance


def plot_trajectories(robot_list, point_size=5):
    ''' Plots robot trajectories as a point cloud with loop closures highlighted. '''
    colors = {
        1: [0, 0.6, 0],  # Green
        2: [1, 0, 1],  # Purple
        3: [1, 0.5, 0],  # Orange
        4: [0, 0, 1]   # Blue
    }
    overlap_color = [1, 0, 0]  # Red for overlapping points

    point_clouds = []

    for robot in robot_list:
        if not robot.poses:
            continue  

        poses = np.array(robot.poses)[:, :3]  
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(poses)

        # Assign colors: Red for overlap, normal for rest
        point_colors = []
        for i in range(len(poses)):
            print(i)
            if i in robot.overlap_indices:
                print("SHIT")
                point_colors.append(overlap_color)
            else:
                point_colors.append(colors[robot.number])

        pcd.colors = o3d.utility.Vector3dVector(point_colors)
        point_clouds.append(pcd)

        # Compute total path length
        path_length = robot.compute_path_length()
        print(f"Total path length for {robot.name}: {path_length:.4f} meters")

        # Compute overlapping path length
        overlap_length = robot.compute_loop_closures(distance_threshold=0.5, min_time_gap=10)
        print(f"Overlapping path length for {robot.name}: {overlap_length:.4f} meters")

    # Create visualization window
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    for pcd in point_clouds:
        vis.add_geometry(pcd)

    # Adjust point sizes
    opt = vis.get_render_option()
    opt.point_size = point_size  
    vis.run()
    vis.destroy_window()


# Initialize robots
robot_nums = [1, 2, 3, 4]
env_name = "main_campus"
robot_list = [Robot(env_name, robot_num) for robot_num in robot_nums]

# Plot trajectories with loop closure highlighting
plot_trajectories(robot_list, point_size=8)
