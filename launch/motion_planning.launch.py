"""
Lance le planificateur A* + le suiveur de chemin.

Ne lance PAS Gazebo ni le simple_mapper : ceux-ci doivent déjà tourner
(via `ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py`).
Ce launch file vient s'ajouter par-dessus, dans un terminal séparé.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('motion_planning_project')
    params_file = os.path.join(pkg_share, 'config', 'planner_params.yaml')

    planner_node = Node(
        package='motion_planning_project',
        executable='planner_node',
        name='planner_node',
        output='screen',
        parameters=[params_file],
    )

    path_follower_node = Node(
        package='motion_planning_project',
        executable='path_follower_node',
        name='path_follower_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        planner_node,
        path_follower_node,
    ])
