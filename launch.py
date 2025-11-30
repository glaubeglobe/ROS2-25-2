from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    gym_pkg = get_package_share_directory('f1tenth_gym_ros')
    gym_launch = os.path.join(gym_pkg, 'launch', 'gym_bridge_launch.py')

    nav2_pkg = get_package_share_directory('nav2_bringup')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')

    this_pkg = get_package_share_directory('f1tenth_raceline_planner')
    nav2_params = os.path.join(this_pkg, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # F1TENTH Gym bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gym_launch),
        ),

        # Nav2 bringup + 우리 planner/controller 설정
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'use_sim_time': 'True',
                'params_file': nav2_params,
            }.items(),
        ),

        # cmd_vel -> drive 브리지
        Node(
            package='f1tenth_raceline_planner',
            executable='twist_to_drive.py',
            name='twist_to_drive',
            output='screen',
            parameters=[{'wheelbase': 0.33}],
        ),

        # perception → costmap 연결 노드
        Node(
            package='f1tenth_raceline_planner',
            executable='tracking_cpp',
            name='tracking_cpp',
            output='screen',
        ),
    ])