# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2022 Intel Corporation. All Rights Reserved.

"""Launch realsense2_camera node without rviz2."""
import os
import sys
import pathlib
import yaml
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument,GroupAction,TimerAction
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir,LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    config_dir = LaunchConfiguration('config_dir', default=os.path.join(get_package_share_directory('realsense2_nav'), 'config'))
    cartographer_configuration_basename = LaunchConfiguration('cartographer_configuration_basename', default='rs_cartographer.lua')
    nav_configuration_filepath = LaunchConfiguration('nav_configuration_filepath', default=os.path.join(get_package_share_directory('realsense2_nav'), 'config/nav2_params.yaml'))

    with open(os.path.join(get_package_share_directory('realsense2_nav'), 'config/config.yml'), 'r') as f:
        params = yaml.safe_load(f)
        t265_base_frame_id=params['t265_base_frame_id']
        rgbd_base_frame_id=params['d435_base_frame_id']
        use_sim_time = params['use_sim_time']
        resolution = params['resolution']
        publish_period_sec = params['publish_period_sec']
        output_topic = params['output_topic']
        D435_x = params['D435_x']
        D435_y = params['D435_y']
        D435_z = params['D435_z']
        D435_roll = params['D435_roll']
        D435_pitch = params['D435_pitch']
        D435_yaw = params['D435_yaw']
        T265_z = params['T265_z']

    t265_static_parameters = {
        'camera_name': 't265',
        'device_type': 't265',
        'base_frame_id': t265_base_frame_id,
        'enable_fisheye1': 'true',
        'enable_fisheye2': 'true',
        'enanle_pose': 'true',
        'tf_publish_rate': '60.0',}

    d435_static_parameters = {
        'camera_name': 'd435', 
        'device_type': 'd4.',
        'base_frame_id': rgbd_base_frame_id,
        'enable_color': 'true',
        'enable_depth': 'true',
        'enable_infra1': 'true',
        'enable_infra2': 'true',
        'pointcloud.enable': 'true',
        'pointcloud.dense': 'true',
        'enable_accel': 'true',
        'enable_gyro': 'true',
        'unite_imu_method': '2',
        'depth_topic': '/d435/camera/depth/image_rect_raw',
        'depth_camera_info_topic': '/d435/camera/depth/camera_info',
        'tf_publish_rate': '60.0',
        'align_depth.enable': 'true'}

    realsense_prefix = get_package_share_directory('realsense2_camera')
    cartographer_prefix = get_package_share_directory('realsense_examples')
    nav_prefix = get_package_share_directory('nav2_bringup')

    t265_to_d435_tf_node= Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.03', '0', '0', '0', t265_base_frame_id, rgbd_base_frame_id]
            )
    
    #nav2
    base_link_tf_node= Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=[D435_x, D435_y, D435_z, D435_yaw, D435_roll, D435_pitch, rgbd_base_frame_id, "base_link"]
            )

    return LaunchDescription(
        [
        DeclareLaunchArgument(
            'config_dir',
            default_value=config_dir,
            description='Full path to config file to load'),

        DeclareLaunchArgument(
            'cartographer_configuration_basename',
            default_value=cartographer_configuration_basename,
            description='Name of lua file for cartographer'),

        DeclareLaunchArgument(
            'nav_configuration_filepath',
            default_value=nav_configuration_filepath,
            description='Name of yaml file for navigation2'),
        
        t265_to_d435_tf_node,base_link_tf_node,

        TimerAction(
            period=0.0,
            actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([realsense_prefix, '/launch/rs_launch.py']),
                launch_arguments=t265_static_parameters.items()
            )
            ]
        ),

        TimerAction(
            period=5.0,
            actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([realsense_prefix, '/launch/rs_launch.py']),
                launch_arguments=d435_static_parameters.items()
            )
            ]
        ),

        TimerAction(
            period=10.0,
            actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([cartographer_prefix, '/launch/rs_cartographer.launch.py']),
                launch_arguments={"cartographer_config_dir":config_dir,"configuration_basename":cartographer_configuration_basename,"resolution":resolution,"publish_period_sec":publish_period_sec}.items()
            )
            ]
        ),

        TimerAction(
            period=15.0,
            actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav_prefix, '/launch/nav2_navigation_launch.py']),
                launch_arguments={"params":nav_configuration_filepath}.items()
            )
            ]
        ),

        Node(
                package='cmd_vel_relay',
                node_executable='relay',
                node_name='relay',
                parameters=[{'input_topic': "cmd_vel",'output_topic': output_topic}])
    ])
