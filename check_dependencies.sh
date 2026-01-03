#!/bin/bash

# Script kiểm tra các thư viện cần thiết cho dự án xe tự hành
# ROS Jazzy + Gazebo Sim 8

echo "=========================================="
echo "KIỂM TRA CÁC THƯ VIỆN DỰ ÁN XE TỰ HÀNH"
echo "=========================================="
echo ""

# Màu sắc cho output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

check_command() {
    if command -v $1 &> /dev/null; then
        echo -e "${GREEN}✓${NC} $1 đã được cài đặt"
        if [ ! -z "$2" ]; then
            VERSION=$($1 $2 2>&1 | head -n 1)
            echo "  └─ Phiên bản: $VERSION"
        fi
        return 0
    else
        echo -e "${RED}✗${NC} $1 chưa được cài đặt"
        return 1
    fi
}

check_ros_package() {
    if ros2 pkg list 2>/dev/null | grep -q "^$1$"; then
        echo -e "${GREEN}✓${NC} $1"
        return 0
    else
        echo -e "${RED}✗${NC} $1"
        return 1
    fi
}

check_python_package() {
    if python3 -c "import $1" 2>/dev/null; then
        echo -e "${GREEN}✓${NC} $1"
        VERSION=$(python3 -c "import $1; print($1.__version__)" 2>/dev/null || echo "N/A")
        echo "  └─ Phiên bản: $VERSION"
        return 0
    else
        echo -e "${RED}✗${NC} $1"
        return 1
    fi
}

# 1. Kiểm tra ROS 2 Jazzy
echo "1. KIỂM TRA ROS 2 JAZZY"
echo "----------------------------------------"
check_command "ros2" "--version"
if [ $? -eq 0 ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || echo -e "${YELLOW}⚠${NC} Không thể source ROS Jazzy"
    echo -e "  ROS_DISTRO: ${GREEN}$ROS_DISTRO${NC}"
fi
echo ""

# 2. Kiểm tra Gazebo Sim 8
echo "2. KIỂM TRA GAZEBO SIM 8"
echo "----------------------------------------"
check_command "gz" "sim --version"
check_command "ign" "gazebo --version"
echo ""

# 3. Kiểm tra các gói ROS 2 cơ bản
echo "3. KIỂM TRA CÁC GÓI ROS 2 CƠ BẢN"
echo "----------------------------------------"
BASIC_PACKAGES=(
    "rclcpp"
    "rclpy"
    "std_msgs"
    "sensor_msgs"
    "geometry_msgs"
    "nav_msgs"
    "tf2"
    "tf2_ros"
    "tf2_geometry_msgs"
)

for pkg in "${BASIC_PACKAGES[@]}"; do
    check_ros_package "$pkg"
done
echo ""

# 4. Kiểm tra Navigation Stack (Nav2)
echo "4. KIỂM TRA NAVIGATION STACK (NAV2)"
echo "----------------------------------------"
NAV2_PACKAGES=(
    "nav2_bringup"
    "nav2_navigation"
    "nav2_map_server"
    "nav2_amcl"
    "nav2_controller"
    "nav2_planner"
    "nav2_costmap_2d"
    "nav2_bt_navigator"
    "nav2_behavior_tree"
)

for pkg in "${NAV2_PACKAGES[@]}"; do
    check_ros_package "$pkg"
done
echo ""

# 5. Kiểm tra SLAM
echo "5. KIỂM TRA SLAM"
echo "----------------------------------------"
SLAM_PACKAGES=(
    "slam_toolbox"
    "cartographer"
    "cartographer_ros"
)

for pkg in "${SLAM_PACKAGES[@]}"; do
    check_ros_package "$pkg"
done
echo ""

# 6. Kiểm tra Sensor Drivers
echo "6. KIỂM TRA SENSOR DRIVERS"
echo "----------------------------------------"
SENSOR_PACKAGES=(
    "velodyne"
    "velodyne_driver"
    "sick_scan"
    "realsense2_camera"
    "usb_cam"
    "image_transport"
    "cv_bridge"
)

for pkg in "${SENSOR_PACKAGES[@]}"; do
    check_ros_package "$pkg"
done
echo ""

# 7. Kiểm tra Robot Description & Simulation
echo "7. KIỂM TRA ROBOT DESCRIPTION & SIMULATION"
echo "----------------------------------------"
ROBOT_PACKAGES=(
    "robot_state_publisher"
    "joint_state_publisher"
    "xacro"
    "urdf"
    "ros_gz_bridge"
    "ros_gz_sim"
    "ros_gz_image"
)

for pkg in "${ROBOT_PACKAGES[@]}"; do
    check_ros_package "$pkg"
done
echo ""

# 8. Kiểm tra Control Packages
echo "8. KIỂM TRA CONTROL PACKAGES"
echo "----------------------------------------"
CONTROL_PACKAGES=(
    "controller_manager"
    "diff_drive_controller"
    "joint_state_broadcaster"
    "ros2_control"
    "ros2_controllers"
)

for pkg in "${CONTROL_PACKAGES[@]}"; do
    check_ros_package "$pkg"
done
echo ""

# 9. Kiểm tra Vision & AI Libraries
echo "9. KIỂM TRA VISION & AI LIBRARIES"
echo "----------------------------------------"
check_command "python3" "--version"
echo ""
echo "Python packages:"
PYTHON_PACKAGES=(
    "cv2"
    "numpy"
    "torch"
    "tensorflow"
    "sklearn"
)

for pkg in "${PYTHON_PACKAGES[@]}"; do
    check_python_package "$pkg"
done
echo ""

# 10. Kiểm tra Build Tools
echo "10. KIỂM TRA BUILD TOOLS"
echo "----------------------------------------"
check_command "colcon" "version"
check_command "cmake" "--version"
check_command "gcc" "--version"
check_command "g++" "--version"
check_command "python3" "--version"
check_command "pip3" "--version"
echo ""

# 11. Kiểm tra Version Control
echo "11. KIỂM TRA VERSION CONTROL"
echo "----------------------------------------"
check_command "git" "--version"
echo ""

# 12. Kiểm tra các công cụ khác
echo "12. KIỂM TRA CÁC CÔNG CỤ KHÁC"
echo "----------------------------------------"
OTHER_PACKAGES=(
    "rviz2"
    "rqt"
    "rqt_graph"
    "rqt_console"
    "rqt_robot_steering"
)

for pkg in "${OTHER_PACKAGES[@]}"; do
    check_ros_package "$pkg"
done
echo ""

# Tóm tắt
echo "=========================================="
echo "TÓM TẮT"
echo "=========================================="
echo -e "${YELLOW}Lưu ý:${NC} Nếu thiếu gói nào, bạn có thể cài đặt bằng:"
echo ""
echo "  # Cài đặt ROS 2 packages:"
echo "  sudo apt install ros-jazzy-<package-name>"
echo ""
echo "  # Cài đặt Gazebo Sim 8:"
echo "  sudo apt install gz-harmonic"
echo ""
echo "  # Cài đặt Python packages:"
echo "  pip3 install <package-name>"
echo ""
echo "  # Cài đặt Nav2:"
echo "  sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup"
echo ""
echo "  # Cài đặt SLAM Toolbox:"
echo "  sudo apt install ros-jazzy-slam-toolbox"
echo ""
echo "  # Cài đặt ros_gz bridge:"
echo "  sudo apt install ros-jazzy-ros-gz"
echo ""
echo "=========================================="
