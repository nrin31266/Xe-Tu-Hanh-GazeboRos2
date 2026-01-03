# 🚗 CD2 Xe Tự Hành - ROS2 Jazzy + Gazebo Sim 8

Xe tự lái né vật cản trên đường thẳng 100m x 8m với 10 vật cản.

## Yêu cầu
- ROS2 Jazzy
- Gazebo Sim 8
- ros_gz_bridge

## Build
```bash
cd ~/cd2_xe_tu_hanh
colcon build
source install/setup.bash
```

## Chạy Simulation
```bash
cd ~/cd2_xe_tu_hanh
source install/setup.bash
ros2 launch cd2_robot run.launch.py
```

Hoặc headless (không GUI):
```bash
ros2 launch cd2_robot run.launch.py enable_gui:=false
```

## Debug Commands

```bash
# Kiểm tra topics
ros2 topic list

# Xem cmd_vel
ros2 topic echo /cmd_vel --once

# Xem LiDAR
ros2 topic echo /scan --once

# Xem odometry
ros2 topic echo /odom --once

# Kiểm tra tần số
ros2 topic hz /cmd_vel

# Kiểm tra nodes
ros2 node list

# Xem Gazebo topics
gz topic -l
gz topic -i -t /cmd_vel
```

## Tham số chính
| Tham số | Giá trị | Mô tả |
|---------|---------|-------|
| forward_speed | 2.0 m/s | Tốc độ tiến |
| safe_distance | 2.5 m | Khoảng cách an toàn |
| max_distance_m | 105 m | Quãng đường tối đa |

## Troubleshooting

**Xe không di chuyển:**
```bash
# Kiểm tra bridge /clock
ros2 topic info /clock
# Phải có Publisher count: 1

# Test pub thủ công
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"
```

**LiDAR không hoạt động:**
- Cần GPU support
- Kiểm tra: `ros2 topic hz /scan`

**Dọn processes cũ:**
```bash
pkill -9 -f "gz sim"; pkill -9 -f "ros2"
```
