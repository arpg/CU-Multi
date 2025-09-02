import os
from math import pi
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def get_robot(root_path, env, robot_name):
    # Root directory path to robot instance
    robot_dir = os.path.join(root_path, env, robot_name)

    # Paths to bags for particular robo
    lidar_path = os.path.join(
        robot_dir, 
        f"{robot_name}_{env}_lidar", 
        f"{robot_name}_{env}_lidar_0.db3"
    )
    poses_bag_path = os.path.join(
        robot_dir,
        f"{robot_name}_{env}_poses_CSV_NEW",
        f"{robot_name}_{env}_poses_CSV_NEW_0.db3"
    )
    # poses_bag_path = os.path.join(
    #     robot_dir,
    #     f"{robot_name}_{env}_poses",
    #     f"{robot_name}_{env}_poses_0.db3"
    # )

    robot_lidar_bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', lidar_path],
        output = 'screen'
    )
    robot_pose_bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', poses_bag_path, '--clock'],
        output = 'screen'
    )

    # Publish URDF for robot
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('lidar2osm_ros'), 'urdf', 'multi_robot.urdf.xacro']
    )
    xacro_command = Command(
        ['xacro ', xacro_file, ' ', 'name:=', f"{robot_name}"]
    )
    robot_urdf = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'{robot_name}_robot_state_publisher',
        parameters=[{'robot_description': xacro_command}],
        remappings=[('/robot_description', f'/{robot_name}/robot_description')]
    )

    # IMU_TO_LIDAR = {
    #     "x": -0.06286,
    #     "y":  0.01557,
    #     "z":  0.053345,
    #     "roll":  0.0,
    #     "pitch": 0.0,
    #     "yaw":   pi,   # 180 deg about Z
    # }

    # # --- NEW: static TF IMU -> LiDAR ---
    # imu_to_lidar_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name=f'{robot_name}_imu_to_lidar_tf',
    #     # args order: x y z roll pitch yaw parent child
    #     arguments=[
    #         str(IMU_TO_LIDAR["x"]), str(IMU_TO_LIDAR["y"]), str(IMU_TO_LIDAR["z"]),
    #         str(IMU_TO_LIDAR["roll"]), str(IMU_TO_LIDAR["pitch"]), str(IMU_TO_LIDAR["yaw"]),
    #         f'{robot_name}_imu_link', f'{robot_name}_os_sensor'
    #     ],
    #     output='screen'
    # )

    return [robot_urdf, robot_pose_bag_play, robot_lidar_bag_play]


def generate_launch_description():
    environment = "main_campus"
    robot_names = ["robot1"]
    root_path = f"/home/donceykong/Data/cu_multi"

    ld = LaunchDescription()

    # sim-time arg
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (bag) time if true'
    ))

    # robot bags + state-publisher
    for robot_name in robot_names:
        for action in get_robot(root_path, environment, robot_name):
            ld.add_action(action)

    # delayed RViz
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('lidar2osm_ros'), 'rviz', 'play_bags.rviz'
        ])]
    )
    ld.add_action(TimerAction(period=5.0, actions=[rviz]))

    return ld
