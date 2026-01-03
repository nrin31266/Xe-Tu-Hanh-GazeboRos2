#!/usr/bin/env python3
"""
NODE NÉ VẬT CẢN (KHÔNG KÉO LÀN)
===============================
- Xe spawn ở làn 2 (y=0)
- Gặp vật cản → rẽ 90° → chạy ngang → rẽ 90° về
- KHÔNG có cơ chế kéo về trung tâm làn
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


# ========== TRẠNG THÁI ==========
CHAY_THANG = 0
LUI_LAI = 1
RE_TRANH_VAT = 2
CHAY_NGANG = 3
RE_VE_HUONG_CU = 4


class AvoidanceNode(Node):
    """Node điều khiển xe né vật cản (không kéo làn)"""

    def __init__(self):
        super().__init__('avoidance_node')

        # ========== THAM SỐ ==========
        self.declare_parameter('toc_do_tien', 1.5)
        self.declare_parameter('toc_do_lui', 0.5)
        self.declare_parameter('toc_do_quay', 1.0)
        self.declare_parameter('khoang_cach_an_toan', 2.0)
        self.declare_parameter('goc_quet_truoc', 15.0)
        self.declare_parameter('thoi_gian_re_tranh', 1.57)
        self.declare_parameter('thoi_gian_re_ve', 1.57)
        self.declare_parameter('thoi_gian_chay_ngang', 1.5)
        self.declare_parameter('thoi_gian_lui', 0.5)
        self.declare_parameter('khoang_cach_toi_da', 105.0)
        self.declare_parameter('tam_xa_lidar', 5.0)

        # Lấy giá trị
        self.toc_do_tien = float(self.get_parameter('toc_do_tien').value)
        self.toc_do_lui = float(self.get_parameter('toc_do_lui').value)
        self.toc_do_quay = float(self.get_parameter('toc_do_quay').value)
        self.khoang_cach_an_toan = float(self.get_parameter('khoang_cach_an_toan').value)
        self.goc_quet_truoc = float(self.get_parameter('goc_quet_truoc').value)
        self.thoi_gian_re_tranh = float(self.get_parameter('thoi_gian_re_tranh').value)
        self.thoi_gian_re_ve = float(self.get_parameter('thoi_gian_re_ve').value)
        self.thoi_gian_chay_ngang = float(self.get_parameter('thoi_gian_chay_ngang').value)
        self.thoi_gian_lui = float(self.get_parameter('thoi_gian_lui').value)
        self.khoang_cach_toi_da = float(self.get_parameter('khoang_cach_toi_da').value)
        self.tam_xa_lidar = float(self.get_parameter('tam_xa_lidar').value)

        # ========== BIẾN TRẠNG THÁI ==========
        self.trang_thai = CHAY_THANG
        self.huong_re = 0              # 1: trái, -1: phải
        self.thoi_diem_bat_dau = 0.0
        self.da_hoan_thanh = False

        # Odometry
        self.vi_tri_x = 0.0
        self.vi_tri_y = 0.0
        self.goc_yaw = 0.0
        self.quang_duong_da_di = 0.0
        self.vi_tri_x_truoc = None
        self.vi_tri_y_truoc = None
        self.da_co_odom = False

        # Scan
        self.du_lieu_scan = None
        self.da_co_scan = False

        # Log
        self.thoi_diem_log = 0.0

        # ========== QoS ==========
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        # ========== SUBSCRIBER / PUBLISHER ==========
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.xu_ly_scan, sensor_qos)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.xu_ly_odom, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer 20Hz
        self.timer = self.create_timer(0.05, self.vong_lap_dieu_khien)

        self.get_logger().info('=' * 55)
        self.get_logger().info('🚗 NODE NÉ VẬT CẢN (KHÔNG KÉO LÀN)')
        self.get_logger().info('=' * 55)

    def xu_ly_scan(self, msg: LaserScan):
        self.du_lieu_scan = msg
        if not self.da_co_scan:
            self.da_co_scan = True
            self.get_logger().info(f'✅ Đã nhận scan! {len(msg.ranges)} điểm')

    def xu_ly_odom(self, msg: Odometry):
        self.vi_tri_x = msg.pose.pose.position.x
        self.vi_tri_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.goc_yaw = math.atan2(siny_cosp, cosy_cosp)

        if not self.da_co_odom:
            self.da_co_odom = True
            self.get_logger().info(
                f'✅ Đã nhận odom! x={self.vi_tri_x:.1f}, y={self.vi_tri_y:.1f}, yaw={math.degrees(self.goc_yaw):.1f}°'
            )

        if self.vi_tri_x_truoc is not None:
            dx = self.vi_tri_x - self.vi_tri_x_truoc
            dy = self.vi_tri_y - self.vi_tri_y_truoc
            self.quang_duong_da_di += math.sqrt(dx*dx + dy*dy)

        self.vi_tri_x_truoc = self.vi_tri_x
        self.vi_tri_y_truoc = self.vi_tri_y

    def kiem_tra_vat_can_truoc(self) -> tuple:
        """Trả về (có_vật, khoảng_cách_min, trống_trái, trống_phải)"""
        if self.du_lieu_scan is None:
            return (False, self.tam_xa_lidar, self.tam_xa_lidar, self.tam_xa_lidar)

        scan = self.du_lieu_scan
        ranges = np.array(scan.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, self.tam_xa_lidar)
        ranges = np.clip(ranges, scan.range_min, self.tam_xa_lidar)

        n = len(ranges)
        if n == 0:
            return (False, self.tam_xa_lidar, self.tam_xa_lidar, self.tam_xa_lidar)

        angles = np.linspace(scan.angle_min, scan.angle_max, n)
        front_rad = math.radians(self.goc_quet_truoc)

        mask_front = np.abs(angles) <= front_rad
        front = ranges[mask_front]
        if front.size == 0:
            return (False, self.tam_xa_lidar, self.tam_xa_lidar, self.tam_xa_lidar)

        min_front = float(np.min(front))

        # trái/phải để chọn hướng
        side_rad = math.radians(45)
        mask_left = (angles > 0) & (angles <= side_rad)
        mask_right = (angles < 0) & (angles >= -side_rad)

        left = ranges[mask_left]
        right = ranges[mask_right]

        gap_left = float(np.percentile(left, 30)) if left.size > 0 else 0.0
        gap_right = float(np.percentile(right, 30)) if right.size > 0 else 0.0

        obstacle = (min_front < self.khoang_cach_an_toan)
        return (obstacle, min_front, gap_left, gap_right)

    def lay_ten_trang_thai(self) -> str:
        ten = ['THẲNG', 'LÙI', 'RẼ_NÉ', 'NGANG', 'RẼ_VỀ']
        return ten[self.trang_thai] if self.trang_thai < len(ten) else '???'

    def vong_lap_dieu_khien(self):
        lenh = Twist()
        t = time.time()
        nen_log = (t - self.thoi_diem_log >= 1.0)

        try:
            if self.da_hoan_thanh:
                self.pub_cmd.publish(lenh)
                return

            if self.da_co_odom and self.quang_duong_da_di >= self.khoang_cach_toi_da:
                self.dung_xe()
                self.da_hoan_thanh = True
                self.get_logger().info(f'🏁 HOÀN THÀNH! Đã đi {self.quang_duong_da_di:.1f}m')
                return

            if not self.da_co_scan or not self.da_co_odom:
                self.pub_cmd.publish(Twist())
                if nen_log:
                    self.get_logger().warn('⏳ Đang chờ scan/odom...')
                    self.thoi_diem_log = t
                return

            co_vat_can, kc, gap_left, gap_right = self.kiem_tra_vat_can_truoc()
            dt_state = t - self.thoi_diem_bat_dau

            if self.trang_thai == CHAY_THANG:
                if co_vat_can:
                    # Chọn hướng rẽ theo làn hiện tại
                    # L1 (y > 1.0): ở trái nhất → rẽ PHẢI
                    # L3 (y < -1.0): ở phải nhất → rẽ TRÁI
                    # L2 (ở giữa): ưu tiên rẽ TRÁI
                    if self.vi_tri_y > 1.0:
                        self.huong_re = -1  # Rẽ PHẢI
                        lan_hien_tai = 'L1'
                    elif self.vi_tri_y < -1.0:
                        self.huong_re = 1   # Rẽ TRÁI
                        lan_hien_tai = 'L3'
                    else:
                        self.huong_re = 1   # Ưu tiên rẽ TRÁI
                        lan_hien_tai = 'L2'

                    if kc < self.khoang_cach_an_toan * 0.5:
                        self.trang_thai = LUI_LAI
                        self.get_logger().info(f'🔙 VẬT QUÁ GẦN ({kc:.2f}m) @ {lan_hien_tai} → LÙI')
                    else:
                        self.trang_thai = RE_TRANH_VAT
                        self.get_logger().info(f'🚧 VẬT CẢN ({kc:.2f}m) @ {lan_hien_tai} → RẼ {"TRÁI" if self.huong_re>0 else "PHẢI"}')

                    self.thoi_diem_bat_dau = t
                else:
                    lenh.linear.x = self.toc_do_tien
                    lenh.angular.z = 0.0

            elif self.trang_thai == LUI_LAI:
                if dt_state < self.thoi_gian_lui:
                    lenh.linear.x = -self.toc_do_lui
                    lenh.angular.z = 0.0
                else:
                    self.trang_thai = RE_TRANH_VAT
                    self.thoi_diem_bat_dau = t
                    self.get_logger().info(f'🔄 LÙI XONG → RẼ {"TRÁI" if self.huong_re>0 else "PHẢI"}')

            elif self.trang_thai == RE_TRANH_VAT:
                if dt_state < self.thoi_gian_re_tranh:
                    lenh.linear.x = 0.0
                    lenh.angular.z = self.huong_re * self.toc_do_quay
                else:
                    self.trang_thai = CHAY_NGANG
                    self.thoi_diem_bat_dau = t
                    self.get_logger().info('➡️ RẼ XONG → CHẠY NGANG')

            elif self.trang_thai == CHAY_NGANG:
                if dt_state < self.thoi_gian_chay_ngang:
                    # nếu đang ngang mà thấy vật sát quá -> rẽ về sớm
                    if co_vat_can and kc < self.khoang_cach_an_toan * 0.6:
                        self.trang_thai = RE_VE_HUONG_CU
                        self.thoi_diem_bat_dau = t
                        self.get_logger().info('⚠️ NGANG GẶP VẬT → RẼ VỀ SỚM')
                    else:
                        lenh.linear.x = self.toc_do_tien
                        lenh.angular.z = 0.0
                else:
                    self.trang_thai = RE_VE_HUONG_CU
                    self.thoi_diem_bat_dau = t
                    self.get_logger().info('🔄 NGANG XONG → RẼ VỀ HƯỚNG CŨ')

            elif self.trang_thai == RE_VE_HUONG_CU:
                if dt_state < self.thoi_gian_re_ve:
                    lenh.linear.x = 0.0
                    lenh.angular.z = -self.huong_re * self.toc_do_quay
                else:
                    self.trang_thai = CHAY_THANG
                    self.huong_re = 0
                    self.get_logger().info('✅ RẼ VỀ XONG → CHẠY THẲNG')

            self.pub_cmd.publish(lenh)

            if nen_log:
                self.get_logger().info(
                    f'🚗 {self.lay_ten_trang_thai():6s} | '
                    f'd={kc:.2f}m | y={self.vi_tri_y:.2f} yaw={math.degrees(self.goc_yaw):.0f}° | '
                    f'v={lenh.linear.x:.2f} w={lenh.angular.z:.2f} | đi={self.quang_duong_da_di:.0f}m'
                )
                self.thoi_diem_log = t

        except Exception as e:
            self.get_logger().error(f'❌ Lỗi: {e}')
            self.pub_cmd.publish(Twist())

    def dung_xe(self):
        self.pub_cmd.publish(Twist())
        self.get_logger().info('🛑 XE ĐÃ DỪNG')


def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('⚡ Ctrl+C → Dừng...')
    finally:
        try:
            node.dung_xe()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
