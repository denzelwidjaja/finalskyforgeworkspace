from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, TimerAction, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Get package paths
    skyforge_pkg = get_package_share_directory('skyforge')

    # Process Xacro to URDF
    xacro_path = os.path.join(skyforge_pkg, 'urdf', 'skyforge_robot.urdf.xacro')
    if not os.path.exists(xacro_path):
        raise FileNotFoundError(f"Could not find xacro file at: {xacro_path}")

    robot_description_config = xacro.process_file(xacro_path)
    robot_description = robot_description_config.toxml()

    # Save URDF to temp for loading in Gazebo
    urdf_temp_path = '/tmp/skyforge_robot.urdf'
    with open(urdf_temp_path, 'w') as f:
        f.write(robot_description)

    # Controller config
    controller_yaml_path = os.path.join(skyforge_pkg, 'config', 'skyforge_controllers.yaml')
    if not os.path.exists(controller_yaml_path):
        raise FileNotFoundError(f"Could not find controller YAML file at: {controller_yaml_path}")

    # Set mesh path for Gazebo
    set_gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(skyforge_pkg)
    )

    return LaunchDescription([
        set_gz_sim_resource_path,

        # Launch Gazebo Sim
        ExecuteProcess(
            cmd=['gz', 'sim', '-v4', '--render-engine', 'ogre2'],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),

        # Spawn robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_skyforge',
            arguments=[
                '-name', 'skyforge',
                '-file', urdf_temp_path,
                '-z', '2.0',
                '--verbose'
            ],
            output='screen',
        ),

        # ROS 2 Control node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description}, controller_yaml_path],
            output='screen',
        ),

        # Delay and then spawn joint_state_broadcaster
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                    output='screen',
                ),
            ]
        ),

        # Delay and then spawn position controller
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['position_controllers', '--controller-manager', '/controller_manager'],
                    output='screen',
                ),
            ]
        ),
    ])
