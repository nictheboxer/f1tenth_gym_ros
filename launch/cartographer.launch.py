# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def generate_launch_description():

    ld = LaunchDescription()

    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    cartographer_config_dir = os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'config')
    configuration_basename = 'cartographer.lua'

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={
            'ign_args': '-r ../gazebo/aapstrack.world'
        }.items(),
    )

    # Bridge 
    # [ from ign to ROS, 
    # ] from ROS to ign, 
    # @ both directional
    ign_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                   '/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                   '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                   '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                   '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                   '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                   '/world/empty/model/rrbot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
                   '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'

                #    '/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
                #    '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'
                   ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'gym_bridge.rviz')]
    )

    config = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'config',
        'sim.yaml'
        )

    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[config]
    )

    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'ego_racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')])

   
    cartographer_node = Node(
            package='cartographer_ros', 
            executable='cartographer_node', 
            name='cartographer_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename])

    occupancy_grid_node = Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        )

    # finalize
    
    ld.add_action(ign_gazebo)
    ld.add_action(rviz_node)
    ld.add_action(ign_bridge)

    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    # ld.add_action(bridge_node)
    # ld.add_action(ego_robot_publisher)

    return ld


