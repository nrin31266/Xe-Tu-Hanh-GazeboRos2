#!/usr/bin/env python3
"""
LAUNCH FILE: Chạy xe tự hành né vật cản
- Khởi động Gazebo với world đường thẳng 100m
- Spawn robot từ SDF model
- Bridge các topic: /cmd_vel, /odom, /scan
- Chạy avoidance_node điều khiển xe

GHI CHÚ:
- KHÔNG dùng URDF vì khi convert sang SDF, gpu_lidar bị đổi thành lidar
- Gazebo Sim 8 KHÔNG hỗ trợ "lidar" sensor, chỉ hỗ trợ "gpu_lidar"
- Model SDF được định nghĩa trong models/cd2_car/model.sdf
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
    
    # 4. Node điều khiển né vật cản + kéo về làn 2
    avoidance_node = Node(
        package='cd2_control',
        executable='avoidance_node',
        name='avoidance_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # Tham số điều khiển
            'toc_do_tien': 1.5,
            'toc_do_lui': 0.5,
            'toc_do_quay': 1.0,
            'khoang_cach_an_toan': 2.0,
            'goc_quet_truoc': 15.0,
            'thoi_gian_re_90': 1.57,
            'thoi_gian_chay_ngang': 1.5,
            'thoi_gian_lui': 0.5,
            'khoang_cach_toi_da': 105.0,
            'tam_xa_lidar': 5.0,
            # Tham số làn đường
            'y_lan_2': 0.0,
            'nguong_y_ok': 0.3,
            'nguong_yaw_ok': 0.1,
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
