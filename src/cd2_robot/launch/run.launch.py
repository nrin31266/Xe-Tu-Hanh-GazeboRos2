#!/usr/bin/env python3
"""
LAUNCH FILE: Chạy xe tự hành né vật cản
- Khởi động Gazebo với world đường thẳng 100m
- Spawn robot từ SDF model (QUAN TRỌNG: dùng SDF để giữ gpu_lidar type!)
- Bridge các topic: /cmd_vel, /odom, /scan
- Chạy avoidance_node điều khiển xe

GHI CHÚ:
- KHÔNG dùng URDF vì khi convert sang SDF, gpu_lidar bị đổi thành lidar
- Gazebo Sim 8 KHÔNG hỗ trợ "lidar" sensor, chỉ hỗ trợ "gpu_lidar"
- Model SDF được định nghĩa trực tiếp trong models/cd2_car/model.sdf
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Lấy đường dẫn package
    pkg_cd2_robot = get_package_share_directory('cd2_robot')
    
    # Đường dẫn file
    world_file = os.path.join(pkg_cd2_robot, 'worlds', 'straight_100x8.world.sdf')
    model_sdf_file = os.path.join(pkg_cd2_robot, 'models', 'cd2_car', 'model.sdf')
    models_path = os.path.join(pkg_cd2_robot, 'models')
    
    # Set GZ_SIM_RESOURCE_PATH để Gazebo tìm được model
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )
    
    # ========== KHAI BÁO CÁC ACTION ==========
    
    # 1. Chạy Gazebo Sim với world và GUI
    # '-r': run ngay (không pause), '-v 3': verbose level 3
    gz_sim = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-r', '-v', '3', world_file
        ],
        output='screen',
        shell=False
    )
    
    # 2. Spawn robot vào Gazebo từ SDF file
    # QUAN TRỌNG: Dùng -file thay vì -topic để spawn từ SDF
    # Sử dụng shell với sleep để đợi Gazebo khởi động xong
    spawn_robot = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'sleep 10 && gz service -s /world/straight_road_world/create '
            f'--reqtype gz.msgs.EntityFactory '
            f'--reptype gz.msgs.Boolean '
            f'--timeout 10000 '
            f'--req \'sdf_filename: "{model_sdf_file}", name: "cd2_car", pose: {{position: {{x: 2.0, y: 0.0, z: 0.3}}}}\''
        ],
        output='screen',
        shell=False
    )
    
    # 3. Bridge các topic giữa Gazebo và ROS2
    # QUAN TRỌNG: Topic PHẢI khớp với plugin trong SDF model
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            # QUAN TRỌNG: Bridge /clock để use_sim_time hoạt động!
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # cmd_vel: ROS -> Gazebo (điều khiển xe) - dùng ] cho ROS->GZ
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # odom: Gazebo -> ROS (vị trí xe)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # scan: Gazebo -> ROS (dữ liệu LiDAR)
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # tf: Gazebo -> ROS
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ]
    )
    
    # 4. Node điều khiển né vật cản
    avoidance_node = Node(
        package='cd2_control',
        executable='avoidance_node',
        name='avoidance_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # Các tham số có thể tune
            'forward_speed': 2.0,
            'safe_distance': 2.5,
            'front_angle_deg': 25.0,
            'min_turn': 0.4,
            'max_turn': 2.2,
            'max_angular_rate_change': 3.0,
            'hysteresis_margin': 0.3,
            'max_distance_m': 105.0,  # Đi hết đường 100m
            'max_range': 10.0
        }]
    )
    
    # ========== TRẢ VỀ LAUNCH DESCRIPTION ==========
    return LaunchDescription([
        # Set environment variable
        set_gz_resource_path,
        # Khởi động theo thứ tự
        gz_sim,
        spawn_robot,
        ros_gz_bridge,
        avoidance_node,
    ])
