import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('motion_planning_project')
    world_file = os.path.join(pkg_share, 'worlds', 'simple_world.world')
    params_file = os.path.join(pkg_share, 'config', 'planner_params.yaml')

    world_arg = DeclareLaunchArgument(
        'world', default_value=world_file, description='Chemin du monde Gazebo à charger'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    planner_node = Node(
        package='motion_planning_project',
        executable='planner_node',
        name='planner_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        world_arg,
        gazebo,
        planner_node,
    ])
