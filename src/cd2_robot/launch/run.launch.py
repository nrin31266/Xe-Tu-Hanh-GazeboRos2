#!/usr/bin/env python3
"""
LAUNCH FILE: Chạy xe tự hành né vật cản
- Khởi động Gazebo với world đường thẳng 100m
- Spawn robot từ SDF model
- Bridge các topic: /cmd_vel, /odom, /scan
- Chạy avoidance_node điều khiển xe né vật cản (KHÔNG kéo làn)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    pkg_cd2_robot = get_package_share_directory('cd2_robot')

    world_file = os.path.join(pkg_cd2_robot, 'worlds', 'straight_100x8.world.sdf')
    model_sdf_file = os.path.join(pkg_cd2_robot, 'models', 'cd2_car', 'model.sdf')
    models_path = os.path.join(pkg_cd2_robot, 'models')

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '3', world_file],
        output='screen',
        shell=False
    )

    # Spawn ở làn 2: x=2, y=0, z=0.25
    spawn_robot = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'sleep 5 && gz service -s /world/straight_road_world/create '
            f'--reqtype gz.msgs.EntityFactory '
            f'--reptype gz.msgs.Boolean '
            f'--timeout 10000 '
            f'--req \'sdf_filename: "{model_sdf_file}", name: "cd2_car", pose: {{position: {{x: 2.0, y: 0.0, z: 0.25}}}}\''
        ],
        output='screen',
        shell=False
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ]
    )

    # Node né vật cản (không kéo làn)
    avoidance_node = Node(
        package='cd2_control',
        executable='avoidance_node',
        name='avoidance_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'toc_do_tien': 2.0,
            'toc_do_lui': 0.5,
            'toc_do_quay': 1.0,
            'khoang_cach_an_toan': 2.0,
            'goc_quet_truoc': 15.0,
            'thoi_gian_re_tranh': 1.0,
            'thoi_gian_re_ve': 0.7065,
            'thoi_gian_chay_ngang': 1.57,
            'thoi_gian_lui': 0.5,
            'khoang_cach_toi_da': 105.0,
            'tam_xa_lidar': 5.0,
        }]
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        spawn_robot,
        ros_gz_bridge,
        avoidance_node,
    ])
