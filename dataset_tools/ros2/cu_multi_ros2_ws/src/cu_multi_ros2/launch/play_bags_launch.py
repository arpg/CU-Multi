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
        f"{robot_name}_{env}_lidar_poses_CSV", 
        f"{robot_name}_{env}_lidar_poses_CSV_0.db3"
    )

    robot_lidar_bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', lidar_path],
        output = 'screen'
    )

    # # Publish URDF for robot
    # xacro_file = PathJoinSubstitution(
    #     [FindPackageShare('lidar2osm_ros'), 'urdf', 'multi_robot.urdf.xacro']
    # )
    # xacro_command = Command(
    #     ['xacro ', xacro_file, ' ', 'name:=', f"{robot_name}"]
    # )
    # robot_urdf = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name=f'{robot_name}_robot_state_publisher',
    #     parameters=[{'robot_description': xacro_command}],
    #     remappings=[('/robot_description', f'/{robot_name}/robot_description')]
    # )

    return [robot_lidar_bag_play]


def generate_launch_description():
    environment = "main_campus"
    robot_names = ["robot1", "robot2"]
    root_path = f"/media/donceykong/doncey_ssd_03/CU_MULTI"

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
