#!/usr/bin/env python3
# =====================================================================================
#  teleop_keyboard.py - Dieu khien robot qua TwistStamped cho DiffDriveController / MobileController
#
#  Phat tin hieu TwistStamped (linear.x, angular.z) toi topic:
#      /mobile_controller/cmd_vel
#
#  PHIM:
#      w : Tang van toc toi (linear.x +)
#      s : Tang van toc lui (linear.x -)
#      a : Tang van toc quay trai (angular.z +)
#      d : Tang van toc quay phai (angular.z -)
#      SPACE : DUNG NGAY (dua ca linear va angular ve 0)
#      q / Ctrl+C : Thoat
# =====================================================================================

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

# Topic xuat lenh van toc TwistStamped
CMD_TOPIC = '/mobile_controller/cmd_vel'

HELP_TEXT = """
--------------------- TELEOP TWIST STAMPED ---------------------
   w : Tang van toc toi      s : Tang van toc lui
   a : Tang quay trai        d : Tang quay phai
   SPACE : DUNG NGAY         q : Thoat
----------------------------------------------------------------
"""


class TeleopKeyboard(Node):
    """Node publish TwistStamped vao topic cmd_vel cua mobile_controller."""

    def __init__(self):
        # [FIX] Khong hardcode use_sim_time=True nua: khi chay Gazebo hay truyen
        # `--ros-args -p use_sim_time:=true`; khi chay robot that/RViz de mac dinh False.
        super().__init__('teleop_keyboard')

        # Tham so buoc tang va gioi han van toc
        self.declare_parameter('linear_step', 0.05)    # m/s moi lan bam
        self.declare_parameter('angular_step', 0.1)    # rad/s moi lan bam
        self.declare_parameter('max_linear_vel', 1.0)  # m/s
        self.declare_parameter('max_angular_vel', 2.0) # rad/s
        self.declare_parameter('publish_rate', 10.0)   # Hz
        self.declare_parameter('frame_id', 'base_link')

        # Vận tốc hiện tại
        self.linear_x = 0.0
        self.angular_z = 0.0

        self.publisher_ = self.create_publisher(TwistStamped, CMD_TOPIC, 10)
        self.create_timer(1.0 / self.get_parameter('publish_rate').value, self.publish_cmd)

    def publish_cmd(self):
        """Dong goi gia tri thanh TwistStamped bao gom Header và Stamp."""
        msg = TwistStamped()
        
        # Gán thời gian hiện tại và frame ID cho header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value

        # Gán vận tốc tuyến tính và vận tốc góc
        msg.twist.linear.x = float(self.linear_x)
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = float(self.angular_z)

        self.publisher_.publish(msg)

    def _clamp(self, val, max_val):
        return max(-max_val, min(max_val, val))

    def update_from_key(self, key):
        lin_step = float(self.get_parameter('linear_step').value)
        ang_step = float(self.get_parameter('angular_step').value)
        max_lin = float(self.get_parameter('max_linear_vel').value)
        max_ang = float(self.get_parameter('max_angular_vel').value)

        if key == 'w':
            self.linear_x = self._clamp(self.linear_x + lin_step, max_lin)
        elif key == 's':
            self.linear_x = self._clamp(self.linear_x - lin_step, max_lin)
        elif key == 'a':
            self.angular_z = self._clamp(self.angular_z + ang_step, max_ang)
        elif key == 'd':
            self.angular_z = self._clamp(self.angular_z - ang_step, max_ang)
        elif key == ' ':
            self.linear_x = 0.0
            self.angular_z = 0.0
        else:
            return False

        self.get_logger().info(
            f'linear.x={self.linear_x:+.2f} m/s | angular.z={self.angular_z:+.2f} rad/s',
            throttle_duration_sec=0.2
        )
        # [FIX] Gui lenh ngay khi bam phim (khong phai cho timer)
        self.publish_cmd()
        return True

    def publish_stop(self):
        self.linear_x = 0.0
        self.angular_z = 0.0
        for _ in range(5):
            self.publish_cmd()
            rclpy.spin_once(self, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()

    if not sys.stdin.isatty():
        node.get_logger().error(
            'stdin khong phai terminal => khong doc duoc ban phim. '
            'Hay chay truc tiep trong terminal.')
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