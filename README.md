# 🚗 CD2 XE TỰ HÀNH - ROS2 Jazzy + Gazebo Sim 8

Xe tự hành 4 bánh né vật cản trên đường thẳng 100m x 8m với 10 vật cản.

## 📋 Mô tả
- **Điều khiển**: Diff-drive (2 bánh sau chủ động)
- **Cảm biến**: LiDAR 360° phía trước
- **Thuật toán**: Reactive avoidance (không stop/lùi)

## Yêu cầu
- ROS2 Jazzy
- Gazebo Sim 8
- ros_gz_bridge

---

## 📁 Cấu trúc dự án

```
cd2_xe_tu_hanh/
├── src/
│   ├── cd2_robot/              # Package robot (ament_cmake)
│   │   ├── urdf/
│   │   │   └── car.urdf.xacro  # Mô hình xe 4 bánh + LiDAR
│   │   ├── worlds/
│   │   │   └── straight_100x8.world.sdf  # World 100m x 8m
│   │   ├── launch/
│   │   │   └── run.launch.py   # Launch file chính
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── cd2_control/            # Package điều khiển (ament_python)
│       ├── cd2_control/
│       │   ├── __init__.py
│       │   ├── avoidance_node.py    # Node né vật cản
│       │   └── simple_radar_gui.py  # GUI radar đơn giản
│       ├── setup.py
│       └── package.xml
│
└── README.md
```

---

## 🗺️ World: straight_100x8

- **Kích thước**: 100m x 8m (biên ±4m)
- **START**: x=0, y=0 (marker xanh lá)
- **GOAL**: x=100, y=0 (marker đỏ)

### 📦 Danh sách vật cản (10 cái)

| Tên          | Loại     | Kích thước         | Tọa độ (x, y, z) |
|--------------|----------|---------------------|------------------|
| obstacle_01  | Box      | 1.2 × 1.2 × 1.2 m  | (15, 0, 0.6)     |
| obstacle_02  | Cylinder | r=0.7m, h=1.2m     | (25, 1.5, 0.6)   |
| obstacle_03  | Box      | 1.4 × 1.4 × 1.2 m  | (35, -1.2, 0.6)  |
| obstacle_04  | Cylinder | r=0.6m, h=1.2m     | (45, 0.8, 0.6)   |
| obstacle_05  | Box      | 1.3 × 1.3 × 1.2 m  | (55, -0.5, 0.6)  |
| obstacle_06  | Cylinder | r=0.8m, h=1.2m     | (65, 1.8, 0.6)   |
| obstacle_07  | Box      | 1.2 × 1.2 × 1.2 m  | (75, -1.5, 0.6)  |
| obstacle_08  | Cylinder | r=0.65m, h=1.2m    | (85, 0.3, 0.6)   |
| obstacle_09  | Box      | 1.4 × 1.4 × 1.2 m  | (92, -1.0, 0.6)  |
| obstacle_10  | Cylinder | r=0.7m, h=1.2m     | (98, 1.2, 0.6)   |

**Ghi chú**: obstacle_01 đặt giữa đường (y=0) để chắc chắn xe gặp. Các vật cản khác phân bố trong vùng y ∈ [-2.0, +2.0].

---

## 📡 Topics quan trọng

| Topic      | Loại                        | Hướng         | Mô tả                    |
|------------|------------------------------|---------------|--------------------------|
| `/cmd_vel` | `geometry_msgs/msg/Twist`   | ROS → Gazebo  | Điều khiển tốc độ xe     |
| `/odom`    | `nav_msgs/msg/Odometry`     | Gazebo → ROS  | Vị trí/vận tốc xe        |
| `/scan`    | `sensor_msgs/msg/LaserScan` | Gazebo → ROS  | Dữ liệu LiDAR 360°       |
| `/clock`   | `rosgraph_msgs/msg/Clock`   | Gazebo → ROS  | Đồng bộ thời gian        |

---

## 🚀 Hướng dẫn chạy

### 1. Build dự án

```bash
cd ~/cd2_xe_tu_hanh

# Dọn sạch (nếu cần)
rm -rf build install log

# Build
colcon build --symlink-install

# Source
source install/setup.bash
```

### 2. Chạy simulation

```bash
ros2 launch cd2_robot run.launch.py
```

Hoặc headless (không GUI Gazebo):
```bash
ros2 launch cd2_robot run.launch.py enable_gui:=false
```

### 3. Chạy Radar GUI (LiDAR Visualizer)

Mở terminal khác khi simulation đang chạy:
```bash
source ~/cd2_xe_tu_hanh/install/setup.bash
ros2 run cd2_control simple_radar_gui
```

### 4. Test điều khiển tay (debug)

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}"
```

---

## 🔍 Debug Commands

```bash
# Kiểm tra topics
ros2 topic list

# Xem cmd_vel / scan / odom
ros2 topic echo /cmd_vel --once
ros2 topic echo /scan --once
ros2 topic echo /odom --once

# Kiểm tra tần số
ros2 topic hz /cmd_vel
ros2 topic hz /scan

# Kiểm tra nodes
ros2 node list

# Xem Gazebo topics
gz topic -l
gz topic -i -t /cmd_vel

# Kiểm tra bridge /clock
ros2 topic info /clock
```

---

## ⚙️ Tham số & Tuning

### Tham số chính

| Tham số | Mặc định | Mô tả |
|---------|----------|-------|
| forward_speed | 2.0 m/s | Tốc độ tiến |
| safe_distance | 2.5 m | Khoảng cách an toàn |
| max_turn | 2.2 rad/s | Tốc độ quay tối đa |
| max_distance_m | 105 m | Quãng đường dừng |

### Hướng dẫn tune

**forward_speed:**
- Tăng (2.5-3.0): Nhanh hơn, cần safe_distance lớn
- Giảm (1.0-1.5): Chậm, an toàn hơn

**safe_distance:**
- Công thức: safe_distance ≈ forward_speed × 1.2 giây
- Tăng (3.0-4.0): Né sớm hơn
- Giảm (1.5-2.0): Đi sát vật cản

**max_turn:**
- Tăng (2.5-3.0): Quay nhanh, có thể giật
- Giảm (1.5-2.0): Quay từ từ, mượt hơn

### Tune runtime (không cần rebuild)

```bash
ros2 param set /avoidance_node forward_speed 2.5
ros2 param set /avoidance_node safe_distance 3.0
ros2 param set /avoidance_node max_turn 2.0
```

---

## 🔧 Troubleshooting

**Xe không di chuyển:**
```bash
ros2 topic info /clock        # Phải có Publisher count: 1
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"
```

**Xe giật/zigzag nhiều:**
- Giảm `max_turn` xuống 1.8-2.0
- Tăng `hysteresis_margin` lên 0.5

**Xe đâm vật cản:**
- Tăng `safe_distance` lên 3.0-3.5
- Giảm `forward_speed` xuống 1.5
- Kiểm tra: `ros2 topic echo /scan`

**LiDAR không hoạt động:**
- Cần GPU support
- Kiểm tra: `ros2 topic hz /scan`

**Dọn processes cũ:**
```bash
pkill -9 -f "gz sim"; pkill -9 -f "ros2"
```

**Gazebo crash:**
- Kiểm tra GPU driver
- Thử: `gz sim -r straight_100x8.world.sdf`

---

## 📝 Ghi chú thuật toán

### Reactive Avoidance
1. Xét vùng trước ±25° (front_angle_deg)
2. Nếu `min_front > safe_distance + hysteresis` → chạy thẳng
3. Nếu `min_front ≤ safe_distance`:
   - Tách vùng trái/phải
   - Tính gap_left, gap_right (percentile 30%)
   - Né về phía gap lớn hơn
   - `closeness = (safe_distance - min_front) / safe_distance`
   - `angular = sign × (min_turn + closeness × (max_turn - min_turn))`
   - `linear giảm: speed × (1 - 0.4 × closeness)`
4. Làm mượt angular bằng ramp
5. Dừng khi `distance_traveled ≥ max_distance_m`

### Vì sao dùng percentile 30%?
- Ổn định hơn min() khi có noise
- Bỏ qua các điểm outlier
- Đánh giá đúng "khoảng trống thực sự" của mỗi bên

---

## 📜 License
MIT
