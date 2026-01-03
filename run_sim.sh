#!/bin/bash
# =============================================================================
# SCRIPT KHỞI ĐỘNG SIMULATION - CHỈ CÓ 1 ENTRYPOINT
# =============================================================================
# Script này là ENTRYPOINT DUY NHẤT để chạy simulation.
# KHÔNG gọi ros2 launch từ nơi khác để tránh duplicate nodes.
#
# Sử dụng:
#   ./run_sim.sh           # Chạy headless (không GUI)
#   ./run_sim.sh --gui     # Chạy với GUI Gazebo
# =============================================================================

set -e  # Dừng nếu có lỗi

# ========== PARSE ARGUMENTS ==========
ENABLE_GUI="false"
for arg in "$@"; do
    case $arg in
        --gui)
            ENABLE_GUI="true"
            shift
            ;;
    esac
done

# ========== DỌN PROCESS CŨ (GUARD CHỐNG DUPLICATE) ==========
echo "🧹 Dọn dẹp processes cũ..."
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "parameter_bridge" 2>/dev/null || true
pkill -9 -f "avoidance" 2>/dev/null || true
pkill -9 -f "ros2 launch cd2_robot" 2>/dev/null || true
pkill -9 -f "cd2_ros_gz_bridge" 2>/dev/null || true
pkill -9 -f "cd2_avoidance_node" 2>/dev/null || true
sleep 2

# ========== SOURCE WORKSPACE ==========
cd ~/cd2_xe_tu_hanh
source install/setup.bash

# ========== THÔNG BÁO ==========
echo ""
echo "=========================================="
echo "🚗 KHỞI ĐỘNG XE TỰ HÀNH NÉ VẬT CẢN"
echo "=========================================="
echo "GUI: $ENABLE_GUI"
echo "World: straight_100x8 (100m x 8m, 10 obstacles)"
echo ""
echo "⏳ Đợi 20 giây để Gazebo + sensors khởi động..."
echo ""

# ========== CHẠY LAUNCH (CHỈ 1 LẦN DUY NHẤT) ==========
ros2 launch cd2_robot run.launch.py enable_gui:=$ENABLE_GUI &
LAUNCH_PID=$!
echo "Launch PID: $LAUNCH_PID"

# ========== ĐỢI KHỞI ĐỘNG ==========
sleep 20

# ========== KIỂM TRA ==========
if ps -p $LAUNCH_PID > /dev/null 2>&1; then
    echo ""
    echo "=========================================="
    echo "✅ SIMULATION ĐANG CHẠY!"
    echo "=========================================="
    echo ""
    echo "Nodes đang chạy:"
    ros2 node list 2>/dev/null | grep -E "cd2|bridge|avoidance" || echo "  (đang khởi động...)"
    echo ""
    echo "📊 Các lệnh debug (chạy trong terminal KHÁC):"
    echo "  ros2 node list                    # Xem danh sách nodes"
    echo "  ros2 topic echo /scan --once      # Test LiDAR"
    echo "  ros2 topic echo /cmd_vel --once   # Test điều khiển"
    echo "  ros2 topic echo /odom --once      # Test odometry"
    echo "  gz topic -i -t /cmd_vel           # Kiểm tra Gazebo nhận cmd_vel"
    echo ""
    echo "Nhấn Ctrl+C để dừng simulation"
    echo ""
    
    # Chờ launch kết thúc
    wait $LAUNCH_PID
else
    echo "❌ Launch thất bại!"
    exit 1
fi
