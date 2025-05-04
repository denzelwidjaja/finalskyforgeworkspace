from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os
import xacro

def generate_launch_description():
    # Get paths
    package_dir = get_package_share_directory('skyforge')
    urdf_xacro_path = os.path.join(package_dir, 'urdf', 'skyforge_robot.urdf.xacro')
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    # Process the xacro file into a URDF string
    robot_description_config = xacro.process_file(urdf_xacro_path)
    robot_description = robot_description_config.toxml()

    # Save the processed URDF to a temporary file (for spawn_entity.py)
    urdf_temp_path = os.path.join('/tmp', 'skyforge_robot.urdf')
    with open(urdf_temp_path, 'w') as f:
        f.write(robot_description)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            # Optional RViz config, or leave it empty to just start
            # arguments=['-d', 'your_config.rviz'],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py'),
            ),
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'skyforge', '-file', urdf_temp_path],
            output='screen',
        ),
    ])
