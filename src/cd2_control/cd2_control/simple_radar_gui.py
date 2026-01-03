#!/usr/bin/env python3
"""
GUI RADAR TỐI GIẢN
==================
Hiển thị dữ liệu LiDAR dạng scatter plot.
Không map/topdown, không animation phức tạp.

Chạy riêng:
  ros2 run cd2_control simple_radar_gui
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt


class SimpleRadarGUI(Node):
    """Node hiển thị radar đơn giản"""
    
    def __init__(self):
        super().__init__('simple_radar_gui')
        
        # Subscribe /scan
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        
        # Setup matplotlib
        plt.ion()  # Interactive mode
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig.canvas.manager.set_window_title('🚗 CD2 Radar - LiDAR View')
        
        # Scatter plot
        self.scatter = self.ax.scatter([], [], c='lime', s=3, alpha=0.8)
        
        # Vẽ xe (tam giác)
        car_x = [0.3, -0.2, -0.2, 0.3]
        car_y = [0, 0.15, -0.15, 0]
        self.ax.fill(car_x, car_y, color='blue', alpha=0.7)
        
        # Cài đặt trục
        self.ax.set_xlim(-12, 12)
        self.ax.set_ylim(-12, 12)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_facecolor('#1a1a2e')
        self.fig.patch.set_facecolor('#16213e')
        
        # Vẽ các vòng tròn khoảng cách
        for r in [2, 4, 6, 8, 10]:
            circle = plt.Circle((0, 0), r, fill=False, color='gray', alpha=0.3, linestyle='--')
            self.ax.add_patch(circle)
            self.ax.text(r + 0.1, 0.1, f'{r}m', fontsize=8, color='gray')
        
        self.ax.set_title('📡 LiDAR Radar View', fontsize=14, color='white', pad=10)
        self.ax.tick_params(colors='white')
        for spine in self.ax.spines.values():
            spine.set_color('white')
        
        plt.tight_layout()
        
        self.get_logger().info('=' * 40)
        self.get_logger().info('📡 GUI RADAR KHỞI ĐỘNG')
        self.get_logger().info('   Đang chờ dữ liệu /scan...')
        self.get_logger().info('   Đóng cửa sổ để thoát.')
        self.get_logger().info('=' * 40)

    def scan_callback(self, msg: LaserScan):
        """Callback nhận và hiển thị dữ liệu LiDAR"""
        # Chuyển ranges thành tọa độ x, y
        ranges = np.array(msg.ranges)
        num_points = len(ranges)
        
        if num_points == 0:
            return
        
        # Tính góc của từng điểm
        angles = np.linspace(msg.angle_min, msg.angle_max, num_points)
        
        # Lọc bỏ inf/nan và giới hạn tầm
        valid_mask = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < 12.0)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        # Chuyển sang tọa độ Cartesian
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        
        # Cập nhật scatter
        self.scatter.set_offsets(np.c_[x, y])
        
        # Vẽ lại
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleRadarGUI()
    
    try:
        # Chạy spin và update GUI
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            
            # Kiểm tra cửa sổ có bị đóng không
            if not plt.fignum_exists(node.fig.number):
                node.get_logger().info('🚪 Cửa sổ đã đóng - Thoát.')
                break
                
    except KeyboardInterrupt:
        node.get_logger().info('⚡ Nhận Ctrl+C - Đang thoát...')
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
