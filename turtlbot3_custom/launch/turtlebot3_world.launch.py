from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get paths to Gazebo and TurtleBot3 worlds
    gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    world_path = os.path.join('/home/arpan/ros2_ws/src/turtlbot3_custom/worlds', 'path2.world')

    # Launch Gazebo with your custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'empty_world.launch.py')
        ),
        launch_arguments={'world': world_path}.items(),
    )

    return LaunchDescription([gazebo])
