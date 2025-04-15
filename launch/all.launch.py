from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
import os

def include_launch(pkg, file):
    try:
        pkg_path = FindPackageShare(pkg).find(pkg)
        full_path = os.path.join(pkg_path, 'launch', file)
        print(f"Including: {full_path}")
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(full_path)
        )
    except Exception as e:
        print(f"⚠️ Failed to include {file} from {pkg}: {e}")
        raise

def generate_launch_description():
    return LaunchDescription([
        include_launch('semantic_navigation', 'turtlebot3_small_house.launch.py'),
        include_launch('turtlebot3_bringup', 'rviz2.launch.py'),
        include_launch('slam_toolbox', 'online_async_launch.py'),
        include_launch('nav2_bringup', 'navigation_launch.py'),
    ])
