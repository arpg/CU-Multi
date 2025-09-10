import os
from math import pi
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def get_robot(DATA_ROOT, env, robot_name):
    # Root directory path to robot instance
    # robot_dir = os.path.join(DATA_ROOT, env, robot_name)

    # # Paths to bags for particular robo
    # lidar_path = os.path.join(
    #     f"{merged_bag}", 
    #     f"{env}_merged_bag_OG_0.db3"
    # )

    # robot_lidar_bag_play = ExecuteProcess(
    #     cmd=['ros2', 'bag', 'play', lidar_path],
    #     output = 'screen'
    # )

    print("\n\nAbout to launch accumulator node\n\n")

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

    return [robot_urdf]


def generate_launch_description():

    DATA_ROOT = os.getenv("CU_MULTI_ROOT")
    if DATA_ROOT is None:
        sys.exit(
            "ERROR: Environment variable CU_MULTI_ROOT is not set.\n"
            "Please set it before running the script, e.g.:\n"
            "  export CU_MULTI_ROOT=/your/custom/path\n"
        )
    print("CU_MULTI_ROOT is:", DATA_ROOT)

    USING_OG = False
    env = "kittredge_loop"
    robot_names = ["robot4"]

    ld = LaunchDescription()

    # sim-time arg
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (bag) time if true'
    ))

    if USING_OG:
        merged_bag = os.path.join(DATA_ROOT, f"{env}", f"{env}_merged_bag_OG")
        merged_bag_path = os.path.join(
            f"{merged_bag}", 
            f"{env}_merged_bag_OG_0.db3"
        )
        merged_bag_play = ExecuteProcess(
            cmd=['ros2', 'bag', 'play', merged_bag_path],
            output = 'screen'
        )
    else:
        merged_bag = os.path.join(DATA_ROOT, f"{env}", f"{env}_robots1_2_3_4_merged_bag")
        merged_bag_path = os.path.join(
            f"{merged_bag}", 
            f"{env}_robots1_2_3_4_merged_bag_0.db3"
        )
        merged_bag_play = ExecuteProcess(
            cmd=['ros2', 'bag', 'play', merged_bag_path],
            output = 'screen'
        )
    ld.add_action(merged_bag_play)

    if USING_OG:
        for robot_name in robot_names:
            for action in get_robot(DATA_ROOT, env, robot_name):
                ld.add_action(action)
    
    for robot_name in robot_names:
        robot_map_accumulator_node = Node(
            package='lidar2osm_ros',
            executable='robot_map_accumulator',
            arguments=[robot_name],  # Pass the robot name as an argument
        )
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
