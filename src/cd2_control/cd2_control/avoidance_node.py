#!/usr/bin/env python3
"""
NODE NÉ VẬT CẢN + KÉO VỀ LÀN 2
===============================
- Xe spawn ở làn 2 (y=0)
- Gặp vật cản → rẽ 90° → chạy ngang → rẽ 90° về
- Sau khi né xong → kéo về làn 2 (y≈0, yaw≈0)

TRẠNG THÁI:
1. CHẠY_THẲNG: Đi thẳng, kiểm tra vật cản
2. LÙI_LẠI: Lùi nếu quá gần
3. RẼ_TRÁNH: Rẽ 90° tránh vật
4. CHẠY_NGANG: Đi qua vật cản
5. RẼ_VỀ: Rẽ 90° về hướng cũ
6. KÉO_VỀ_LÀN: Quay + di chuyển về làn 2
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
KEO_VE_LAN = 5


class AvoidanceNode(Node):
    """Node điều khiển xe né vật cản + kéo về làn 2"""
    
    def __init__(self):
        super().__init__('avoidance_node')
        
        # ========== THAM SỐ ==========
        self.declare_parameter('toc_do_tien', 1.5)
        self.declare_parameter('toc_do_lui', 0.5)
        self.declare_parameter('toc_do_quay', 1.0)
        self.declare_parameter('khoang_cach_an_toan', 2.0)
        self.declare_parameter('goc_quet_truoc', 15.0)
        self.declare_parameter('thoi_gian_re_90', 1.57)       # π/2 / toc_do_quay
        self.declare_parameter('thoi_gian_chay_ngang', 1.5)
        self.declare_parameter('thoi_gian_lui', 0.5)
        self.declare_parameter('khoang_cach_toi_da', 105.0)
        self.declare_parameter('tam_xa_lidar', 5.0)
        self.declare_parameter('y_lan_2', 0.0)               # Vị trí y của làn 2
        self.declare_parameter('nguong_y_ok', 0.3)           # Sai số y chấp nhận được
        self.declare_parameter('nguong_yaw_ok', 0.1)         # Sai số yaw chấp nhận được (rad)
        
        # Lấy giá trị
        self.toc_do_tien = self.get_parameter('toc_do_tien').value
        self.toc_do_lui = self.get_parameter('toc_do_lui').value
        self.toc_do_quay = self.get_parameter('toc_do_quay').value
        self.khoang_cach_an_toan = self.get_parameter('khoang_cach_an_toan').value
        self.goc_quet_truoc = self.get_parameter('goc_quet_truoc').value
        self.thoi_gian_re_90 = self.get_parameter('thoi_gian_re_90').value
        self.thoi_gian_chay_ngang = self.get_parameter('thoi_gian_chay_ngang').value
        self.thoi_gian_lui = self.get_parameter('thoi_gian_lui').value
        self.khoang_cach_toi_da = self.get_parameter('khoang_cach_toi_da').value
        self.tam_xa_lidar = self.get_parameter('tam_xa_lidar').value
        self.y_lan_2 = self.get_parameter('y_lan_2').value
        self.nguong_y_ok = self.get_parameter('nguong_y_ok').value
        self.nguong_yaw_ok = self.get_parameter('nguong_yaw_ok').value
        
        # ========== BIẾN TRẠNG THÁI ==========
        self.trang_thai = CHAY_THANG
        self.huong_re = 0              # 1: trái, -1: phải
        self.thoi_diem_bat_dau = 0.0
        self.da_hoan_thanh = False
        
        # Odometry
        self.vi_tri_x = 0.0
        self.vi_tri_y = 0.0
        self.goc_yaw = 0.0             # Góc quay hiện tại (rad)
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
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.xu_ly_scan, sensor_qos
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.xu_ly_odom, 10
        )
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer 20Hz
        self.timer = self.create_timer(0.05, self.vong_lap_dieu_khien)
        
        # ========== IN THÔNG TIN ==========
        self.get_logger().info('=' * 55)
        self.get_logger().info('🚗 NODE NÉ VẬT CẢN + KÉO VỀ LÀN 2')
        self.get_logger().info('=' * 55)
        self.get_logger().info(f'📊 THAM SỐ:')
        self.get_logger().info(f'   Tốc độ: tiến={self.toc_do_tien}, lùi={self.toc_do_lui}, quay={self.toc_do_quay}')
        self.get_logger().info(f'   Khoảng cách an toàn: {self.khoang_cach_an_toan}m')
        self.get_logger().info(f'   Thời gian: rẽ90={self.thoi_gian_re_90}s, ngang={self.thoi_gian_chay_ngang}s')
        self.get_logger().info(f'   Làn 2: y={self.y_lan_2}, ngưỡng y={self.nguong_y_ok}m, yaw={self.nguong_yaw_ok}rad')
        self.get_logger().info('=' * 55)

    def xu_ly_scan(self, msg: LaserScan):
        """Nhận dữ liệu LiDAR"""
        self.du_lieu_scan = msg
        if not self.da_co_scan:
            self.da_co_scan = True
            self.get_logger().info(f'✅ Đã nhận scan! {len(msg.ranges)} điểm')

    def xu_ly_odom(self, msg: Odometry):
        """Nhận dữ liệu odometry - lấy vị trí x, y và góc yaw"""
        self.vi_tri_x = msg.pose.pose.position.x
        self.vi_tri_y = msg.pose.pose.position.y
        
        # Tính góc yaw từ quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.goc_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        if not self.da_co_odom:
            self.da_co_odom = True
            self.get_logger().info(f'✅ Đã nhận odom! x={self.vi_tri_x:.1f}, y={self.vi_tri_y:.1f}, yaw={math.degrees(self.goc_yaw):.1f}°')
        
        # Tính quãng đường
        if self.vi_tri_x_truoc is not None:
            dx = self.vi_tri_x - self.vi_tri_x_truoc
            dy = self.vi_tri_y - self.vi_tri_y_truoc
            self.quang_duong_da_di += math.sqrt(dx*dx + dy*dy)
        
        self.vi_tri_x_truoc = self.vi_tri_x
        self.vi_tri_y_truoc = self.vi_tri_y

    def kiem_tra_vat_can_truoc(self) -> tuple:
        """Kiểm tra vật cản phía trước. Trả về (có_vật, khoảng_cách, trống_trái, trống_phải)"""
        if self.du_lieu_scan is None:
            return (False, self.tam_xa_lidar, self.tam_xa_lidar, self.tam_xa_lidar)
        
        scan = self.du_lieu_scan
        ranges = np.array(scan.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, self.tam_xa_lidar)
        ranges = np.clip(ranges, scan.range_min, self.tam_xa_lidar)
        
        so_diem = len(ranges)
        if so_diem == 0:
            return (False, self.tam_xa_lidar, self.tam_xa_lidar, self.tam_xa_lidar)
        
        goc = np.linspace(scan.angle_min, scan.angle_max, so_diem)
        goc_quet_rad = math.radians(self.goc_quet_truoc)
        
        # Vùng trước
        mask_truoc = np.abs(goc) <= goc_quet_rad
        ranges_truoc = ranges[mask_truoc]
        
        if len(ranges_truoc) == 0:
            return (False, self.tam_xa_lidar, self.tam_xa_lidar, self.tam_xa_lidar)
        
        khoang_cach_min = np.min(ranges_truoc)
        
        # Vùng trái/phải (góc rộng hơn để quyết định hướng né)
        goc_ben = math.radians(45)
        mask_trai = (goc > 0) & (goc <= goc_ben)
        mask_phai = (goc < 0) & (goc >= -goc_ben)
        
        ranges_trai = ranges[mask_trai]
        ranges_phai = ranges[mask_phai]
        
        trong_trai = np.percentile(ranges_trai, 30) if len(ranges_trai) > 0 else 0
        trong_phai = np.percentile(ranges_phai, 30) if len(ranges_phai) > 0 else 0
        
        co_vat_can = khoang_cach_min < self.khoang_cach_an_toan
        
        return (co_vat_can, khoang_cach_min, trong_trai, trong_phai)

    def lay_ten_trang_thai(self) -> str:
        """Lấy tên trạng thái để log"""
        ten = ['THẲNG', 'LÙI', 'RẼ_NÉ', 'NGANG', 'RẼ_VỀ', 'KÉO_LÀN']
        return ten[self.trang_thai] if self.trang_thai < len(ten) else '???'

    def vong_lap_dieu_khien(self):
        """Vòng lặp điều khiển chính - 20Hz"""
        lenh = Twist()
        t = time.time()
        nen_log = (t - self.thoi_diem_log >= 1.0)
        
        try:
            # Đã hoàn thành?
            if self.da_hoan_thanh:
                self.pub_cmd.publish(lenh)
                return
            
            # Đủ quãng đường?
            if self.da_co_odom and self.quang_duong_da_di >= self.khoang_cach_toi_da:
                self.dung_xe()
                self.da_hoan_thanh = True
                self.get_logger().info(f'🏁 HOÀN THÀNH! Đã đi {self.quang_duong_da_di:.1f}m')
                return
            
            # Chưa có scan/odom -> chờ
            if not self.da_co_scan or not self.da_co_odom:
                lenh.linear.x = 0.0
                self.pub_cmd.publish(lenh)
                if nen_log:
                    self.get_logger().warn('⏳ Đang chờ scan/odom...')
                    self.thoi_diem_log = t
                return
            
            # Lấy thông tin
            co_vat_can, khoang_cach, trong_trai, trong_phai = self.kiem_tra_vat_can_truoc()
            thoi_gian_da_chay = t - self.thoi_diem_bat_dau
            
            # Tính sai số so với làn 2
            sai_y = self.vi_tri_y - self.y_lan_2
            sai_yaw = self.goc_yaw  # Mục tiêu yaw = 0
            
            # ========== MÁY TRẠNG THÁI ==========
            
            if self.trang_thai == CHAY_THANG:
                # Kiểm tra cần kéo về làn không (khi đang THẲNG)
                can_keo_ve = abs(sai_y) > self.nguong_y_ok or abs(sai_yaw) > self.nguong_yaw_ok
                
                if co_vat_can:
                    # Ưu tiên né vật cản trước
                    if trong_trai >= trong_phai:
                        self.huong_re = 1  # Trái
                    else:
                        self.huong_re = -1  # Phải
                    
                    if khoang_cach < self.khoang_cach_an_toan * 0.5:
                        self.trang_thai = LUI_LAI
                        self.get_logger().info(f'🔙 VẬT QUÁ GẦN ({khoang_cach:.1f}m) → LÙI')
                    else:
                        self.trang_thai = RE_TRANH_VAT
                        huong_str = "TRÁI" if self.huong_re > 0 else "PHẢI"
                        self.get_logger().info(f'🚧 PHÁT HIỆN VẬT ({khoang_cach:.1f}m) → RẼ {huong_str}')
                    self.thoi_diem_bat_dau = t
                    
                elif can_keo_ve:
                    # Kéo về làn 2
                    self.trang_thai = KEO_VE_LAN
                    self.thoi_diem_bat_dau = t
                    self.get_logger().info(f'🎯 LỆCH LÀN (y={sai_y:.2f}, yaw={math.degrees(sai_yaw):.1f}°) → KÉO VỀ')
                else:
                    # Đi thẳng
                    lenh.linear.x = self.toc_do_tien
                    lenh.angular.z = 0.0
            
            elif self.trang_thai == LUI_LAI:
                if thoi_gian_da_chay < self.thoi_gian_lui:
                    lenh.linear.x = -self.toc_do_lui
                    lenh.angular.z = 0.0
                else:
                    self.trang_thai = RE_TRANH_VAT
                    self.thoi_diem_bat_dau = t
                    self.get_logger().info(f'🔄 LÙI XONG → RẼ {"TRÁI" if self.huong_re > 0 else "PHẢI"}')
            
            elif self.trang_thai == RE_TRANH_VAT:
                if thoi_gian_da_chay < self.thoi_gian_re_90:
                    lenh.linear.x = 0.0
                    lenh.angular.z = self.huong_re * self.toc_do_quay
                else:
                    self.trang_thai = CHAY_NGANG
                    self.thoi_diem_bat_dau = t
                    self.get_logger().info('➡️ RẼ XONG → CHẠY NGANG')
            
            elif self.trang_thai == CHAY_NGANG:
                if thoi_gian_da_chay < self.thoi_gian_chay_ngang:
                    # Kiểm tra vật cản khi đang chạy ngang
                    if co_vat_can and khoang_cach < self.khoang_cach_an_toan * 0.6:
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
                if thoi_gian_da_chay < self.thoi_gian_re_90:
                    lenh.linear.x = 0.0
                    lenh.angular.z = -self.huong_re * self.toc_do_quay
                else:
                    self.trang_thai = CHAY_THANG
                    self.huong_re = 0
                    self.get_logger().info('✅ RẼ VỀ XONG → CHẠY THẲNG')
            
            elif self.trang_thai == KEO_VE_LAN:
                # Logic kéo về làn 2:
                # 1. Nếu yaw sai nhiều → quay về yaw=0 trước
                # 2. Nếu y sai nhiều → di chuyển ngang (kết hợp tiến + quay)
                # 3. Nếu ok → về CHẠY_THẲNG
                
                if co_vat_can and khoang_cach < self.khoang_cach_an_toan:
                    # Có vật cản → dừng kéo, né trước
                    self.trang_thai = CHAY_THANG
                    self.get_logger().info('⚠️ ĐANG KÉO GẶP VẬT → DỪNG KÉO')
                elif abs(sai_y) <= self.nguong_y_ok and abs(sai_yaw) <= self.nguong_yaw_ok:
                    # Đã về làn 2
                    self.trang_thai = CHAY_THANG
                    self.get_logger().info(f'✅ ĐÃ VỀ LÀN 2! (y={self.vi_tri_y:.2f}, yaw={math.degrees(self.goc_yaw):.1f}°)')
                else:
                    # Điều khiển P đơn giản
                    # Muốn về y=0: nếu y > 0 → cần đi sang phải (angular âm khi tiến)
                    # Công thức: quay để hướng về làn + tiến
                    
                    # Góc mục tiêu để về làn
                    goc_huong_ve_lan = math.atan2(-sai_y, 2.0)  # Hướng về y=0, tiến 2m
                    sai_goc = goc_huong_ve_lan - self.goc_yaw
                    
                    # Normalize góc về [-π, π]
                    while sai_goc > math.pi:
                        sai_goc -= 2 * math.pi
                    while sai_goc < -math.pi:
                        sai_goc += 2 * math.pi
                    
                    # Điều khiển P
                    kp_quay = 1.5
                    kp_tien = 0.8
                    
                    lenh.angular.z = kp_quay * sai_goc
                    lenh.angular.z = max(-self.toc_do_quay, min(self.toc_do_quay, lenh.angular.z))
                    
                    # Tiến chậm khi đang quay nhiều
                    if abs(sai_goc) > 0.3:
                        lenh.linear.x = self.toc_do_tien * 0.3
                    else:
                        lenh.linear.x = self.toc_do_tien * kp_tien
            
            # Publish lệnh
            self.pub_cmd.publish(lenh)
            
            # Log
            if nen_log:
                self.get_logger().info(
                    f'🚗 {self.lay_ten_trang_thai():6s} | '
                    f'd={khoang_cach:.1f}m | '
                    f'y={self.vi_tri_y:.2f} yaw={math.degrees(self.goc_yaw):.0f}° | '
                    f'v={lenh.linear.x:.1f} w={lenh.angular.z:.1f} | '
                    f'đi={self.quang_duong_da_di:.0f}m'
                )
                self.thoi_diem_log = t
                
        except Exception as e:
            self.get_logger().error(f'❌ Lỗi: {e}')
            self.pub_cmd.publish(Twist())

    def dung_xe(self):
        """Dừng xe"""
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
        except:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
