#!/usr/bin/env python3
"""
NODE NÉ VẬT CẢN REACTIVE - CHỈ 1 THUẬT TOÁN DUY NHẤT
=====================================================
Xe chạy thẳng nhanh, khi gặp vật cản thì né mượt về phía trống hơn.
KHÔNG stop, KHÔNG lùi, KHÔNG 2 tầng.

THUẬT TOÁN:
1) Xét vùng trước: ±front_angle_deg
2) Nếu trống -> chạy thẳng nhanh
3) Nếu có vật cản -> né về hướng trống hơn (trái/phải)
4) Làm mượt angular bằng ramp (tránh giật)
5) Dừng khi đi đủ max_distance_m (nếu có odom)

QUAN TRỌNG:
- Timer 20Hz LUÔN publish cmd_vel (không phụ thuộc callback)
- Nếu chưa có scan -> publish cmd_vel=0 + log
- Nếu chưa có odom -> vẫn chạy (không chặn)
"""

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class AvoidanceNode(Node):
    """Node điều khiển xe né vật cản reactive"""
    
    def __init__(self):
        super().__init__('avoidance_node')
        
        # ========== KHAI BÁO THAM SỐ ==========
        self.declare_parameter('forward_speed', 2.0)          # Tốc độ tiến (m/s)
        self.declare_parameter('safe_distance', 2.5)          # Khoảng cách an toàn (m)
        self.declare_parameter('front_angle_deg', 25.0)       # Góc quét trước (độ)
        self.declare_parameter('min_turn', 0.4)               # Góc quay tối thiểu (rad/s)
        self.declare_parameter('max_turn', 2.2)               # Góc quay tối đa (rad/s)
        self.declare_parameter('max_angular_rate_change', 3.0)  # Tốc độ thay đổi angular (rad/s²)
        self.declare_parameter('hysteresis_margin', 0.3)      # Biên dư để chống giật (m)
        self.declare_parameter('max_distance_m', 105.0)       # Khoảng cách tối đa (m)
        self.declare_parameter('max_range', 10.0)             # Tầm xa LiDAR tính toán (m)
        
        # Lấy giá trị tham số
        self.forward_speed = self.get_parameter('forward_speed').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.front_angle_deg = self.get_parameter('front_angle_deg').value
        self.min_turn = self.get_parameter('min_turn').value
        self.max_turn = self.get_parameter('max_turn').value
        self.max_angular_rate_change = self.get_parameter('max_angular_rate_change').value
        self.hysteresis_margin = self.get_parameter('hysteresis_margin').value
        self.max_distance_m = self.get_parameter('max_distance_m').value
        self.max_range = self.get_parameter('max_range').value
        
        # ========== BIẾN TRẠNG THÁI ==========
        self.current_angular = 0.0      # Góc quay hiện tại (để làm mượt)
        self.distance_traveled = 0.0    # Quãng đường đã đi
        self.last_x = None              # Vị trí x trước đó
        self.last_y = None              # Vị trí y trước đó
        self.is_finished = False        # Đã đi xong chưa
        self.has_odom = False           # Đã nhận odom chưa
        
        self.latest_scan = None         # Dữ liệu scan mới nhất
        self.last_log_time = 0.0        # Thời điểm log gần nhất (để log mỗi 1s)
        self.log_interval = 1.0         # Log mỗi 1 giây
        
        # Biến lưu giá trị cuối để log
        self.last_min_front = 0.0
        self.last_linear_x = 0.0
        self.last_target_angular = 0.0
        self.scan_count = 0  # Đếm số lần nhận scan
        
        # ========== QoS PROFILE ==========
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Sensor data dùng BEST_EFFORT
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # ========== SUBSCRIBER / PUBLISHER ==========
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, sensor_qos  # Dùng sensor_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer điều khiển (20Hz) - LUÔN CHẠY, LUÔN PUBLISH
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # ========== IN THAM SỐ KHI KHỞI ĐỘNG ==========
        self.get_logger().info('=' * 50)
        self.get_logger().info('🚗 NODE NÉ VẬT CẢN KHỞI ĐỘNG')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'📊 THAM SỐ:')
        self.get_logger().info(f'   - Tốc độ tiến: {self.forward_speed} m/s')
        self.get_logger().info(f'   - Khoảng cách an toàn: {self.safe_distance} m')
        self.get_logger().info(f'   - Góc quét trước: ±{self.front_angle_deg}°')
        self.get_logger().info(f'   - Góc quay min/max: {self.min_turn}/{self.max_turn} rad/s')
        self.get_logger().info(f'   - Tốc độ thay đổi angular: {self.max_angular_rate_change} rad/s²')
        self.get_logger().info(f'   - Biên dư hysteresis: {self.hysteresis_margin} m')
        self.get_logger().info(f'   - Quãng đường tối đa: {self.max_distance_m} m')
        self.get_logger().info(f'   - Tầm LiDAR tính toán: {self.max_range} m')
        self.get_logger().info('=' * 50)
        self.get_logger().info('⏳ Đang chờ dữ liệu scan...')

    def scan_callback(self, msg: LaserScan):
        """Callback nhận dữ liệu LiDAR"""
        self.latest_scan = msg
        self.scan_count += 1
        if self.scan_count == 1:
            self.get_logger().info(f'✅ Đã nhận scan đầu tiên! {len(msg.ranges)} điểm')

    def odom_callback(self, msg: Odometry):
        """Callback nhận dữ liệu odometry - tính quãng đường đã đi"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        if not self.has_odom:
            self.has_odom = True
            self.get_logger().info('✅ Đã nhận odom!')
        
        if self.last_x is not None and self.last_y is not None:
            # Tính khoảng cách di chuyển
            dx = x - self.last_x
            dy = y - self.last_y
            dist = math.sqrt(dx * dx + dy * dy)
            self.distance_traveled += dist
        
        self.last_x = x
        self.last_y = y

    def control_loop(self):
        """
        Vòng điều khiển chính - chạy ở 20Hz
        LUÔN PUBLISH CMD_VEL - không return sớm mà không publish
        """
        cmd = Twist()
        current_time = time.time()
        should_log = (current_time - self.last_log_time >= self.log_interval)
        
        try:
            # Kiểm tra đã hoàn thành chưa (chỉ khi có odom)
            if self.is_finished:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                return
            
            # Kiểm tra đủ quãng đường chưa (chỉ khi có odom)
            if self.has_odom and self.distance_traveled >= self.max_distance_m:
                self.stop_robot()
                self.is_finished = True
                self.get_logger().info('=' * 50)
                self.get_logger().info(f'🏁 HOÀN THÀNH! Đã đi {self.distance_traveled:.2f}m')
                self.get_logger().info('=' * 50)
                return
            
            # Nếu chưa có scan -> vẫn chạy thẳng (mode test không LiDAR)
            if self.latest_scan is None:
                cmd.linear.x = self.forward_speed * 0.5
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                if should_log:
                    self.get_logger().warn(f'⚠️ CHƯA CÓ SCAN - chạy thẳng chậm | linear={cmd.linear.x:.2f}')
                    self.last_log_time = current_time
                return
            
            # Tính toán cmd_vel từ scan
            cmd = self.compute_velocity(self.latest_scan)
            
            # LUÔN PUBLISH
            self.cmd_pub.publish(cmd)
            
            # Log mỗi 1 giây 
            if should_log:
                odom_status = f"traveled={self.distance_traveled:.1f}m" if self.has_odom else "no_odom"
                self.get_logger().info(
                    f'🚗 CMD | front={self.last_min_front:.2f}m | '
                    f'v={cmd.linear.x:.2f} | w={cmd.angular.z:.2f} | {odom_status}'
                )
                self.last_log_time = current_time
                
        except Exception as e:
            self.get_logger().error(f'❌ Lỗi control_loop: {e}')
            # Vẫn publish cmd=0 khi có lỗi để xe dừng
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)

    def compute_velocity(self, scan: LaserScan) -> Twist:
        """
        THUẬT TOÁN NÉ VẬT CẢN DUY NHẤT
        ===============================
        1) Xét vùng trước ±front_angle_deg
        2) Tìm min_front trong vùng đó
        3) Nếu trống -> chạy thẳng
        4) Nếu có vật cản -> né về phía trống hơn
        5) Làm mượt angular bằng ramp
        """
        cmd = Twist()
        
        # Chuyển ranges thành numpy array, thay inf/nan bằng max_range
        ranges = np.array(scan.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, self.max_range)
        ranges = np.clip(ranges, scan.range_min, self.max_range)
        
        num_points = len(ranges)
        if num_points == 0:
            # Không có dữ liệu -> chạy thẳng (an toàn hơn đứng yên)
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0
            self.last_min_front = self.max_range
            self.last_linear_x = self.forward_speed
            self.last_target_angular = 0.0
            return cmd
        
        # Tính góc của từng điểm
        angles = np.linspace(scan.angle_min, scan.angle_max, num_points)
        
        # ========== BƯỚC 1: XÉT VÙNG TRƯỚC ==========
        front_angle_rad = math.radians(self.front_angle_deg)
        
        # Lấy các điểm trong vùng trước (-front_angle đến +front_angle)
        front_mask = np.abs(angles) <= front_angle_rad
        front_ranges = ranges[front_mask]
        
        if len(front_ranges) == 0:
            # Không có dữ liệu vùng trước -> chạy thẳng
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0
            self.last_min_front = self.max_range
            self.last_linear_x = self.forward_speed
            self.last_target_angular = 0.0
            return cmd
        
        # Khoảng cách gần nhất phía trước
        min_front = np.min(front_ranges)
        self.last_min_front = min_front
        
        # ========== BƯỚC 2: QUYẾT ĐỊNH HÀNH ĐỘNG ==========
        dt = 0.05  # 20Hz
        target_angular = 0.0
        linear_speed = self.forward_speed
        
        threshold = self.safe_distance + self.hysteresis_margin
        
        if min_front > threshold:
            # TRỐNG - Chạy thẳng nhanh
            target_angular = 0.0
            linear_speed = self.forward_speed
        else:
            # CÓ VẬT CẢN - Cần né
            # Tách vùng trước thành trái và phải
            left_mask = (angles > 0) & (angles <= front_angle_rad)
            right_mask = (angles < 0) & (angles >= -front_angle_rad)
            
            left_ranges = ranges[left_mask]
            right_ranges = ranges[right_mask]
            
            # Tính khoảng trống trái/phải (dùng percentile 30% để ổn định)
            if len(left_ranges) > 0:
                gap_left = np.percentile(left_ranges, 30)
            else:
                gap_left = 0.0
            
            if len(right_ranges) > 0:
                gap_right = np.percentile(right_ranges, 30)
            else:
                gap_right = 0.0
            
            # Chọn hướng né (về phía trống hơn)
            if gap_left >= gap_right:
                sign = 1.0   # Né sang trái (angular dương)
            else:
                sign = -1.0  # Né sang phải (angular âm)
            
            # Tính độ gấp của việc né (càng gần vật cản càng quay mạnh)
            closeness = (self.safe_distance - min_front) / self.safe_distance
            closeness = max(0.0, min(1.0, closeness))
            
            # Tính góc quay mục tiêu
            target_angular = sign * (self.min_turn + closeness * (self.max_turn - self.min_turn))
            
            # Giảm tốc nhẹ khi né (không dừng hẳn)
            speed_factor = 1.0 - 0.4 * closeness
            linear_speed = self.forward_speed * speed_factor
        
        # ========== BƯỚC 3: LÀM MƯỢT ANGULAR (RAMP) ==========
        # Giới hạn tốc độ thay đổi angular
        max_change = self.max_angular_rate_change * dt
        delta = target_angular - self.current_angular
        
        if abs(delta) > max_change:
            delta = max_change if delta > 0 else -max_change
        
        self.current_angular += delta
        
        # ========== BƯỚC 4: GÁN VÀ TRẢ VỀ ==========
        cmd.linear.x = linear_speed
        cmd.angular.z = self.current_angular
        
        # Lưu để log
        self.last_linear_x = linear_speed
        self.last_target_angular = self.current_angular
        
        return cmd

    def stop_robot(self):
        """Dừng xe"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        self.get_logger().info('🛑 XE ĐÃ DỪNG')


def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('⚡ Nhận Ctrl+C - Đang dừng...')
    finally:
        try:
            node.stop_robot()
        except:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
