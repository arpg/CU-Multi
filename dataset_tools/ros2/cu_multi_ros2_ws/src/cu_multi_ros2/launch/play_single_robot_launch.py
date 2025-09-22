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

    DATA_ROOT = os.getenv("CU_MULTI_ROOT")
    if DATA_ROOT is None:
        sys.exit(
            "ERROR: Environment variable CU_MULTI_ROOT is not set.\n"
            "Please set it before running the script, e.g.:\n"
            "  export CU_MULTI_ROOT=/your/custom/path\n"
        )
    print("CU_MULTI_ROOT is:", DATA_ROOT)

    env = "kittredge_loop"
    robot_names = ["robot1"]

    ld = LaunchDescription()

    # sim-time arg
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (bag) time if true'
    ))
    
    for robot_name in robot_names:
        merged_bag = os.path.join(DATA_ROOT, 
                                  f"{env}", 
                                  f"{robot_name}", 
                                  f"{robot_name}_{env}_camera_rgb")
        lidar_path = os.path.join(
            f"{merged_bag}", 
            f"{robot_name}_{env}_camera_rgb.db3"
        )

        robot_lidar_bag_play = ExecuteProcess(
            cmd=['ros2', 'bag', 'play', lidar_path],
            output = 'screen'
        )
        ld.add_action(robot_lidar_bag_play)

        # robot_map_accumulator_node = Node(
        #     package='cu_multi_ros',
        #     executable='robot_map_accumulator',
        #     arguments=[robot_name],  # Pass the robot name as an argument
        # )
        # ld.add_action(robot_map_accumulator_node)

    
    # delayed RViz
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('cu_multi_ros'), 'rviz', 'play_cam.rviz'
        ])]
    )
    ld.add_action(TimerAction(period=5.0, actions=[rviz]))

    return ld
