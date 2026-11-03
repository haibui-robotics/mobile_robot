#!/usr/bin/env python3
"""Test VONG KIN tren ROBOT THAT: cmd_vel -> ESP32/motor -> encoder -> /joint_states -> /odom.

CHAY (sau khi da launch mobile_bringup robot.launch.xml):
    source install/setup.bash
    ros2 run my_mobilerobot wheel_test

MUC DICH / CACH DOC KET QUA:
  A. Lenh DI THANG 0.15 m/s => CA 2 BANH phai CUNG DAU va ~ +1.5 rad/s.
     - Neu 2 banh NGUOC DAU => loi dao chieu motor (xem khoi CHIEU QUAY trong
       esp32_tb6612_jga25_370.ino): phai sua sao cho MOTOR_SIGN_x * ENC_SIGN_x
       doi nhau giua 2 ben.
     - Neu odom x KHONG gan bang 0.225 m => kiem tra wheel_radius / gear_ratio /
       encoder_cpr (URDF) hoac MAX_WHEEL_RAD_S (firmware).
  B. Lenh QUAY TRAI ang=+0.5 rad/s => banh TRAI AM, banh PHAI DUONG (CCW theo URDF),
     yaw trong odom phai ~ +1.0 rad. Neu nguoc => anh xa trai/phai bi hoan doi
     (MyRobotHardware phai tra joint theo TEN, khong dung index 0/1).
  C. Uoc luong lai MAX_WHEEL_RAD_S cua firmware tu van toc do duoc.

CANH BAO AN TOAN: dat robot cho co ~0.5 m trong, bai test nay cho xe chay that.

Lich su: [TAO 2026-10-13] dung de phat hien + kiem chung 2 loi dau tien cua robot that
(2 motor gan doi xung cap cung polarity -> xe quay tai cho; va hoan doi trai/phai).
"""
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

CMD_TOPIC = '/diff_drive_controller/cmd_vel'
ODOM_TOPIC = '/diff_drive_controller/odom'
WHEEL_R = 0.1          # ban kinh banh (m) - khop controller.yaml
SEP = 0.45             # khoang cach tam 2 banh (m) - khop controller.yaml
MAX_RAD_S_CFG = 10.0   # MAX_WHEEL_RAD_S hien tai trong firmware
V_LIN = 0.15           # (m/s) van toc thi nghiem di thang
W_ANG = 0.5            # (rad/s) van toc goc thi nghiem quay
T_LIN = 1.5            # (s)
T_ANG = 2.0            # (s)


def yaw_of(q):
    """Trich yaw (rad) tu quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WheelTest(Node):
    def __init__(self):
        super().__init__('wheel_test')
        self.rec = {}
        self.odom = None
        self.pub = self.create_publisher(TwistStamped, CMD_TOPIC, 10)
        self.create_subscription(JointState, '/joint_states', self._on_js, 50)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_od, 50)

    def _on_js(self, msg):
        for name, vel in zip(msg.name, msg.velocity):
            if name in self.rec:
                self.rec[name].append(vel)

    def _on_od(self, msg):
        self.odom = msg

    def _publish(self, lin, ang):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()   # stamp MOI moi lan gui
        m.header.frame_id = 'base_footprint'
        m.twist.linear.x = float(lin)
        m.twist.angular.z = float(ang)
        self.pub.publish(m)

    def drive(self, lin, ang, dur, label, quiet=False):
        """Gui lenh lien tuc trong `dur` giay, tra ve van toc banh trung vi nua sau."""
        self.rec = {'left_wheel_joint': [], 'right_wheel_joint': []}
        t0 = time.time()
        while time.time() - t0 < dur:
            self._publish(lin, ang)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)
        out = {}
        for name in self.rec:
            tail = self.rec[name][len(self.rec[name]) // 2:] or [float('nan')]
            out[name] = statistics.median(tail)
        if not quiet:
            wL = (lin - ang * SEP / 2.0) / WHEEL_R
            wR = (lin + ang * SEP / 2.0) / WHEEL_R
            self.get_logger().info(
                f'[{label}] cmd lin={lin:+.2f} m/s ang={ang:+.2f} rad/s ({dur:.1f}s)')
            self.get_logger().info(
                f'   LEFT  : do duoc {out["left_wheel_joint"]:+.3f}  / mong doi {wL:+.3f} rad/s')
            self.get_logger().info(
                f'   RIGHT : do duoc {out["right_wheel_joint"]:+.3f}  / mong doi {wR:+.3f} rad/s')
        return out

    def odom_str(self):
        if self.odom is None:
            return 'chua co odom'
        p = self.odom.pose.pose.position
        return (f'x={p.x:+.4f} y={p.y:+.4f} '
                f'yaw={yaw_of(self.odom.pose.pose.orientation):+.4f} rad')


def main():
    rclpy.init()
    node = WheelTest()
    log = node.get_logger()
    log.info('Doi /joint_states + /odom (toi da 10s)...')
    t0 = time.time()
    while node.odom is None and time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.odom is None:
        log.error('Khong nhan duoc /odom - kiem tra: ros2 control list_controllers')
        rclpy.shutdown()
        return

    log.info('########## A. DI THANG (%.2f m/s) ##########' % V_LIN)
    node.drive(0.0, 0.0, 0.8, 'dung yen', quiet=True)
    log.info(f'   odom TRUOC: {node.odom_str()}')
    a = node.drive(V_LIN, 0.0, T_LIN, 'DI THANG')
    node.drive(0.0, 0.0, 0.8, 'dung lai', quiet=True)
    log.info(f'   odom SAU  : {node.odom_str()}')

    log.info('########## B. QUAY TRAI (ang=+%.2f rad/s) ##########' % W_ANG)
    log.info(f'   odom TRUOC: {node.odom_str()}')
    b = node.drive(0.0, W_ANG, T_ANG, 'QUAY +z')
    node.drive(0.0, 0.0, 0.8, 'dung lai', quiet=True)
    log.info(f'   odom SAU  : {node.odom_str()}')

    log.info('########## C. UOC LUONG MAX_WHEEL_RAD_S ##########')
    pwm = int(V_LIN / WHEEL_R / MAX_RAD_S_CFG * 255.0)
    log.info(f'   firmware dang dat MAX_WHEEL_RAD_S = {MAX_RAD_S_CFG}; PWM khi cmd '
             f'{V_LIN / WHEEL_R:.1f} rad/s = {pwm}')
    for name, v in (('LEFT', a['left_wheel_joint']), ('RIGHT', a['right_wheel_joint'])):
        if abs(v) > 0.05:
            est = abs(v) * MAX_RAD_S_CFG / (V_LIN / WHEEL_R)
            log.info(f'   {name}: |v|={abs(v):.3f} rad/s @PWM{pwm}  => MAX ≈ {est:.1f} rad/s')
        else:
            log.info(f'   {name}: KHONG quay ({v:+.3f}) => kiem tra driver/motor!')

    log.info('########## KET LUAN TU DONG ##########')
    ok_dir = (a['left_wheel_joint'] * a['right_wheel_joint'] > 0
              and abs(a['left_wheel_joint']) > 0.05
              and abs(a['right_wheel_joint']) > 0.05)
    if ok_dir:
        log.info('   [OK] 2 banh CUNG CHIEU khi ra lenh di thang -> dung chieu 2 ben.')
    else:
        log.warn('   [LOI] 2 banh NGUOC CHIEU khi ra lenh di thang -> sua dau motor/encoder!')
    if b['left_wheel_joint'] < 0 < b['right_wheel_joint']:
        log.info('   [OK] Quay +z: banh trai am / banh phai duong (CCW) -> dung convention URDF.')
    else:
        log.warn('   [CANH BAO] Quay +z nhung chieu banh KHONG dung -> kiem tra anh xa trai/phai.')

    node.drive(0.0, 0.0, 0.5, 'STOP', quiet=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

