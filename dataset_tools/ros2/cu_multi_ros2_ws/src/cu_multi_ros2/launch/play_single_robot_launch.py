import os
from math import pi
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    USING_OG = True
    env = "main_campus"
    robot_names = ["robot1"]
    root_path = f"/home/donceykong/Data/cu_multi"

    ld = LaunchDescription()

    # sim-time arg
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (bag) time if true'
    ))
    
    for robot_name in robot_names:
        merged_bag = os.path.join(root_path, 
                                  f"{env}", 
                                  f"{robot_name}", 
                                  f"{robot_name}_{env}_lidar_poses_CSV")
        lidar_path = os.path.join(
            f"{merged_bag}", 
            f"{robot_name}_{env}_lidar_poses_CSV_0.db3"
        )

        robot_lidar_bag_play = ExecuteProcess(
            cmd=['ros2', 'bag', 'play', lidar_path],
            output = 'screen'
        )

        robot_map_accumulator_node = Node(
            package='lidar2osm_ros',
            executable='robot_map_accumulator',
            arguments=[robot_name],  # Pass the robot name as an argument
        )
        ld.add_action(robot_lidar_bag_play)
        ld.add_action(robot_map_accumulator_node)

    
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
