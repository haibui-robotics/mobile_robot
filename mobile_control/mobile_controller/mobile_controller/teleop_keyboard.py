#!/usr/bin/env python3
# =====================================================================================
#  teleop_keyboard.py - Dieu khien TRUC TIEP van toc 2 banh cho
#  JointGroupVelocityController (velocity_controllers/JointGroupVelocityController).
#
#  KHAC VOI diff_drive_controller:
#    - diff_drive_controller nhan TwistStamped (linear.x, angular.z) roi TU tinh
#      dong hoc nghich ra van toc 2 banh, dong thoi phat /odom + TF.
#    - JointGroupVelocityController KHONG lam gi ca: no chi nhan 1 mang
#      std_msgs/Float64MultiArray roi gan THANG vao command_interface/velocity
#      cua tung khop theo DUNG THU TU khai bao trong tham so `joints`.
#
#  => Chinh NODE NAY phai tu tinh van toc banh trai/phai.
#     data[0] = van toc left_wheel_joint  (rad/s)
#     data[1] = van toc right_wheel_joint (rad/s)
#
#  CHAY:
#      ros2 launch my_mobilerobot gazebo.launch.xml
#      ros2 run my_mobilerobot teleop_keyboard
#
#  PHIM:
#      w : 2 banh cung +  (di thang toi)
#      s : 2 banh cung -  (lui)
#      a : trai - / phai + (quay tai cho sang trai)
#      d : trai + / phai - (quay tai cho sang phai)
#      SPACE : dung (ca 2 ve 0)
#      q / Ctrl+C : thoat (tu dung truoc khi thoat)
# =====================================================================================

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray

# Topic ma JointGroupVelocityController lang nghe: <ten_controller>/commands
CMD_TOPIC = '/simple_velocity_controller/commands'

HELP_TEXT = """
--------------------- TELEOP JOINT GROUP VELOCITY ---------------------
   w : di thang toi         s : lui
   a : quay tai cho trai    d : quay tai cho phai
   SPACE : DUNG NGAY        q : thoat
------------------------------------------------------------------------
"""


class TeleopKeyboard(Node):
    """Node publish Float64MultiArray vao topic lenh cua JointGroupVelocityController."""

    def __init__(self):
        # [NOTE] Khong hardcode use_sim_time=True de khi chay robot that (khong co Gazebo /clock)
        # thi timer va cac lenh bam phim van duoc publish binh thuong.
        super().__init__('teleop_keyboard')

        # ---- Tham so chinh duoc: --ros-args -p vel_step:=0.5 -p max_vel:=6.0 ----
        self.declare_parameter('vel_step', 0.5)     # rad/s moi lan bam
        self.declare_parameter('max_vel', 6.0)      # rad/s (cho phep chay nhanh den ~60 RPM)
        self.declare_parameter('publish_rate', 10.0)

        # Van toc 2 banh hien tai (rad/s). Thu tu PHAI khop `joints` trong controller.yaml:
        #   joints: [left_wheel_joint, right_wheel_joint]  =>  data = [left, right]
        self.left = 0.0
        self.right = 0.0

        self.publisher_ = self.create_publisher(Float64MultiArray, CMD_TOPIC, 10)
        self.create_timer(1.0 / self.get_parameter('publish_rate').value, self.publish_cmd)

    def publish_cmd(self):
        """Dong goi [left, right] thanh Float64MultiArray roi gui."""
        msg = Float64MultiArray()
        msg.data = [float(self.left), float(self.right)]
        self.publisher_.publish(msg)

    def _clamp(self, v):
        m = float(self.get_parameter('max_vel').value)
        return max(-m, min(m, v))

    def update_from_key(self, key):
        step = float(self.get_parameter('vel_step').value)
        if key == 'w':                       # di thang: 2 banh cung dau (+)
            self.left = self._clamp(self.left + step)
            self.right = self._clamp(self.right + step)
        elif key == 's':                     # lui: 2 banh cung dau (-)
            self.left = self._clamp(self.left - step)
            self.right = self._clamp(self.right - step)
        elif key == 'a':                     # quay trai: trai -, phai +
            self.left = self._clamp(self.left - step)
            self.right = self._clamp(self.right + step)
        elif key == 'd':                     # quay phai: trai +, phai -
            self.left = self._clamp(self.left + step)
            self.right = self._clamp(self.right - step)
        elif key == ' ':
            self.left = 0.0
            self.right = 0.0
        else:
            return False

        self.get_logger().info(
            f'left={self.left:+.2f} rad/s | right={self.right:+.2f} rad/s',
            throttle_duration_sec=0.2)
        # [NOTE] Gui lenh ngay khi bam phim (khong can cho timer)
        self.publish_cmd()
        return True

    def publish_stop(self):
        self.left = 0.0
        self.right = 0.0
        for _ in range(5):
            self.publish_cmd()
            rclpy.spin_once(self, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()

    if not sys.stdin.isatty():
        node.get_logger().error(
            'stdin khong phai terminal => khong doc duoc ban phim. '
            'Hay chay truc tiep trong terminal, hoac dung node demo.')
        node.destroy_node()
        rclpy.shutdown()
        return

    settings = termios.tcgetattr(sys.stdin)
    print(HELP_TEXT)

    try:
        tty.setcbreak(sys.stdin.fileno())
        running = True
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1)
                if key in ('q', '\x03'):
                    running = False
                else:
                    node.update_from_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Dang dung robot va thoat...')
        node.publish_stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
