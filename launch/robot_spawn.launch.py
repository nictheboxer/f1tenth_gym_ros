
import os, yaml, xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def load_file(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None



def generate_launch_description():
    # Import the model urdf (load from file, xacro ...)
    
    xacro_file = get_package_file('f1tenth_gym_ros', 'description/racecar.xacro')
    urdf = xacro.process_file(xacro_file).toprettyxml(indent='  ')
    # robot_desc = load_file(urdf_file)

    # Robot state publisher
    params = {'use_sim_time': True, 'robot_description': urdf}
    robot_state_publisher =  Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params],
            arguments=[])

    # Ignition gazebo
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={'ign_args': '-r aapstrack.world'}.items(),
    )

    # RViz
    pkg_ros_ign_gazebo_demos = get_package_share_directory('ros_ign_gazebo_demos')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_ros_ign_gazebo_demos, 'rviz', 'robot_description_publisher.rviz')]
    )

    # Spawn
    spawn = Node(package='ros_ign_gazebo', executable='create',
                 arguments=[
                    '-name', 'racecar',
                    '-x', '1.2',
                    '-z', '2.3',
                    '-Y', '3.4',
                    '-topic', '/robot_description'],
                 output='screen')

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        rviz,
        spawn
    ])