#!/bin/bash
# ====================================
# SCRIPT DỌN SẠCH VÀ BUILD DỰ ÁN
# ====================================
# Chạy: ./clean_build.sh
# Hoặc: bash clean_build.sh

echo "========================================"
echo "🧹 DỌN SẠCH DỰ ÁN CD2 XE TỰ HÀNH"
echo "========================================"

cd ~/cd2_xe_tu_hanh

echo "📁 Đang dọn build/install/log..."
rm -rf build install log

echo "📁 Đang dọn __pycache__ và *.pyc..."
find src -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null
find src -name "*.pyc" -delete 2>/dev/null

echo "========================================"
echo "🔨 BUILD DỰ ÁN"
echo "========================================"
colcon build --symlink-install

echo "========================================"
echo "✅ HOÀN TẤT!"
echo "========================================"
echo ""
echo "Chạy lệnh sau để source:"
echo "  source install/setup.bash"
echo ""
echo "Sau đó chạy simulation:"
echo "  ros2 launch cd2_robot run.launch.py"
echo ""
