# 🚗 CD2 XE TỰ HÀNH - DỰ ÁN NÉ VẬT CẢN

## 📋 Mô tả
Dự án xe tự hành 4 bánh mô phỏng trong Gazebo Sim 8, sử dụng LiDAR để phát hiện và né vật cản.
- **Điều khiển**: Diff-drive (2 bánh sau chủ động)
- **Cảm biến**: LiDAR 360° phía trước
- **Thuật toán**: Reactive avoidance (1 kiểu duy nhất, không stop/lùi)

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
│       ├── resource/
│       ├── setup.py
│       ├── setup.cfg
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

**Ghi chú**:
- `obstacle_01` đặt giữa đường (y=0) để chắc chắn xe gặp
- Các vật cản khác phân bố trong vùng y ∈ [-2.0, +2.0]
- Khoảng cách giữa các vật cản: 10-15m

---

## 📡 Topics quan trọng

| Topic      | Loại                        | Hướng         | Mô tả                    |
|------------|------------------------------|---------------|--------------------------|
| `/cmd_vel` | `geometry_msgs/msg/Twist`   | ROS → Gazebo  | Điều khiển tốc độ xe     |
| `/odom`    | `nav_msgs/msg/Odometry`     | Gazebo → ROS  | Vị trí/vận tốc xe        |
| `/scan`    | `sensor_msgs/msg/LaserScan` | Gazebo → ROS  | Dữ liệu LiDAR 360°       |

---

## 🚀 Hướng dẫn chạy

### 1. Build dự án

```bash
cd ~/cd2_xe_tu_hanh

# Dọn sạch (QUAN TRỌNG - tránh dính code cũ)
rm -rf build install log
find src -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null
find src -name "*.pyc" -delete 2>/dev/null

# Build
colcon build --symlink-install

# Source
source install/setup.bash
```

### 2. Chạy simulation

```bash
ros2 launch cd2_robot run.launch.py
```

Lệnh này sẽ:
1. Khởi động Gazebo với world đường thẳng
2. Spawn xe tại vị trí START (x=2, y=0)
3. Chạy bridge các topic
4. Chạy avoidance_node điều khiển xe

### 3. Test điều khiển tay (debug)

```bash
# Mở terminal mới, source và chạy:
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}"
```
Nếu xe chạy → bridge hoạt động đúng.

### 4. Chạy GUI Radar (tùy chọn)

```bash
# Mở terminal mới
source ~/cd2_xe_tu_hanh/install/setup.bash
ros2 run cd2_control simple_radar_gui
```

---

## ⚙️ Hướng dẫn tune tham số

### 3 tham số quan trọng nhất:

#### 1. `forward_speed` (mặc định: 2.0 m/s)
- **Tăng** (2.5-3.0): Xe chạy nhanh hơn, cần safe_distance lớn hơn
- **Giảm** (1.0-1.5): Xe chạy chậm, an toàn hơn
- **Khuyên**: Bắt đầu với 1.5, tăng dần khi thuật toán ổn định

#### 2. `safe_distance` (mặc định: 2.5 m)
- **Tăng** (3.0-4.0): Xe phát hiện vật cản sớm hơn, né sớm
- **Giảm** (1.5-2.0): Xe đi sát vật cản hơn, nguy hiểm hơn
- **Công thức**: safe_distance ≈ forward_speed × 1.2 (1.2 giây phản ứng)

#### 3. `max_turn` (mặc định: 2.2 rad/s)
- **Tăng** (2.5-3.0): Xe quay nhanh hơn, né gấp hơn, có thể giật
- **Giảm** (1.5-2.0): Xe quay từ từ, mượt hơn, cần safe_distance lớn
- **Khuyên**: Giữ 2.0-2.5, tăng max_angular_rate_change nếu muốn mượt hơn

### Tune trong launch file:

Sửa file `src/cd2_robot/launch/run.launch.py`:

```python
avoidance_node = Node(
    package='cd2_control',
    executable='avoidance_node',
    parameters=[{
        'forward_speed': 2.5,      # Tăng tốc độ
        'safe_distance': 3.0,      # Tăng khoảng cách an toàn
        'max_turn': 2.0,           # Giảm góc quay max
        # ... các tham số khác
    }]
)
```

### Tune runtime (không cần rebuild):

```bash
ros2 param set /avoidance_node forward_speed 2.5
ros2 param set /avoidance_node safe_distance 3.0
ros2 param set /avoidance_node max_turn 2.0
```

---

## 🔧 Troubleshooting

### Xe không chạy khi pub /cmd_vel
1. Kiểm tra bridge đang chạy: `ros2 topic list`
2. Kiểm tra topic đúng dấu "/": `/cmd_vel` không phải `cmd_vel`
3. Rebuild và source lại

### Xe giật/zigzag nhiều
- Giảm `max_turn` xuống 1.8-2.0
- Tăng `max_angular_rate_change` lên 4.0-5.0
- Tăng `hysteresis_margin` lên 0.5

### Xe đâm vật cản
- Tăng `safe_distance` lên 3.0-3.5
- Giảm `forward_speed` xuống 1.5
- Kiểm tra /scan có dữ liệu: `ros2 topic echo /scan`

### Gazebo crash
- Kiểm tra GPU driver
- Thử chạy không headless: `gz sim -r straight_100x8.world.sdf`

---

## 📝 Ghi chú kỹ thuật

### Thuật toán né vật cản
1. Xét vùng trước ±25° (front_angle_deg)
2. Nếu min_front > safe_distance + hysteresis → chạy thẳng
3. Nếu min_front ≤ safe_distance:
   - Tách vùng trái/phải
   - Tính gap_left, gap_right (percentile 30%)
   - Né về phía gap lớn hơn
   - Closeness = (safe_distance - min_front) / safe_distance
   - Angular = sign × (min_turn + closeness × (max_turn - min_turn))
   - Linear giảm nhẹ: speed × (1 - 0.4 × closeness)
4. Làm mượt angular bằng ramp (max_angular_rate_change)
5. Dừng khi distance_traveled ≥ max_distance_m

### Vì sao dùng percentile 30%?
- Ổn định hơn min() khi có noise
- Bỏ qua các điểm outlier
- Đánh giá đúng hơn "khoảng trống thực sự" của mỗi bên

---

## 📜 License
MIT
