# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2022 Intel Corporation. All Rights Reserved.

"""Launch realsense2_camera node without rviz2."""
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import rs_launch
t265_base_frame_id = 't265_link'#LaunchConfiguration('base_frame_id', default='t265_link')
rgbd_base_frame_id = 'd435_link'#LaunchConfiguration('base_frame_id', default='d435_link')

local_parameters = [{'name': 'camera_name1', 'default': 'D435', 'description': 'camera unique name'},
                    {'name': 'device_type1', 'default': 'd4.', 'description': 'choose device by type'},
                    {'name': 'base_frame_id1', 'default': rgbd_base_frame_id, 'description': 'frame id'},
                    {'name': 'camera_name2', 'default': 'T265', 'description': 'camera unique name'},
                    {'name': 'device_type2', 'default': 't265', 'description': 'choose device by type'},
                    {'name': 'base_frame_id2', 'default': t265_base_frame_id, 'description': 'frame id'},
		    {'name': 'enable_infra11', 'default': 'true', 'description': 'topic for D435 infrared camera'},
		    {'name': 'enable_infra12', 'default': 'true', 'description': 'topic for D435 infrared camera'},
		    {'name': 'pointcloud.enable1', 'default': 'true', 'description': 'topic for D435 depth camera'},
		    {'name': 'pointcloud.dense1', 'default': 'true', 'description': 'topic for D435 depth camera'},
                    {'name': 'enable_fisheye12', 'default': 'true', 'description': 'enable fisheye1 stream'},
                    {'name': 'enable_fisheye22', 'default': 'true', 'description': 'enable fisheye2 stream'},
                    {'name': 'enable_accel2', 'default': 'true', 'description': 'enable accelerometer'},
                    {'name': 'enable_gyro2', 'default': 'true', 'description': 'enable gyroscope'},
                    {'name': 'enanle_pose2', 'default': 'true', 'description': 'enable pose'},
                   ]

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    realsense_prefix = get_package_share_directory('realsense2_camera')
    nav2_prefix = get_package_share_directory('nav2_bringup')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(realsense_prefix, 'launch'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='rs_cartographer.lua')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    rviz_config_dir = os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_cartographer.rviz')
    rviz_node = Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            output = 'screen',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': 'false'}]
            )

    tf_node1= Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', "T265_pose_frame", t265_base_frame_id]
            )
    tf_node2= Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.03', '0', '0', '0', t265_base_frame_id, rgbd_base_frame_id]
            )
    tf_node3= Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '1.57079633', '3.14', '1.57079633', rgbd_base_frame_id, "D435_depth_optical_frame"]
            )
    tf_node4= Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', "map", "odom_frame"]
            )
    tf_node5= Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', t265_base_frame_id, "camera_pose_optical_frame"]
            )
    tf_node6= Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', rgbd_base_frame_id, "base_link"]
            )
    tf_node7= Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', "odom_frame", "odom"]
            ) 

    depthimage_to_laserscan_node= Node(
            package='depthimage_to_laserscan',
            node_executable='depthimage_to_laserscan_node',
            node_name='scan',
            output='screen',
            parameters=[{'output_frame':'T265_pose_frame'}],
            remappings=[('depth','/D435/depth/image_rect_raw'),('depth_camera_info', '/D435/depth/camera_info')]
            )

    cartographer_node= Node(
            package='cartographer_ros',
            node_executable='cartographer_node',
            output='log',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir, '-configuration_basename', configuration_basename],
            remappings=[('odom','/T265/pose/sample'),('points2','/D435/depth/color/points')]
            )

    occupancy_grid_node=Node(
            package='cartographer_ros',
            node_executable='occupancy_grid_node',
            node_name='occupancy_grid_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) + 
        [
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),

        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rs_multi_camera_launch.py']),
            launch_arguments=rs_launch.set_configurable_parameters(local_parameters).items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_prefix, '/launch/nav2_navigation_launch.py'])
        ),

        rviz_node,tf_node1,tf_node2,tf_node3,tf_node4,tf_node5,tf_node6,tf_node7,depthimage_to_laserscan_node,cartographer_node,occupancy_grid_node
    ])
