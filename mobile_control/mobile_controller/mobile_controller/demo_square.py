#!/usr/bin/env python3
# =====================================================================================
#  demo_square.py - Demo VONG HO (open-loop) cho JointGroupVelocityController.
#
#  Vi JointGroupVelocityController KHONG phat /odom, node nay KHONG the dung phan hoi
#  vi tri de biet "da di bao xa / da quay bao nhieu". Thay vao do no chay theo THOI GIAN:
#    - Di thang: [v, v]  trong straight_time giay
#    - Quay tai cho: [-w, w] trong turn_time giay
#    - Lap lai num_sides lan (mo phong hinh vuong) roi dung.
#
#  Day la diem KHAC BIET lon nhat khi hoc ros2_control:
#    diff_drive_controller         = co dong hoc + odom (vong kin)
#    JointGroupVelocityController  = chi "truyen lenh" (vong ho), moi tinh toan o ngoai.
#
#  CHAY:
#      ros2 launch my_mobilerobot gazebo.launch.xml
#      ros2 run my_mobilerobot demo_square
# =====================================================================================

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray

CMD_TOPIC = '/simple_velocity_controller/commands'


class DemoSquare(Node):
    def __init__(self):
        # [NOTE] Khong hardcode use_sim_time=True de khi chay robot that van hoat dong
        super().__init__('demo_square')

        self.declare_parameter('straight_vel', 0.3)   # rad/s khi di thang
        self.declare_parameter('turn_vel', 0.5)       # rad/s khi quay tai cho
        self.declare_parameter('straight_time', 2.0)  # giay di thang
        self.declare_parameter('turn_time', 1.5)      # giay quay
        self.declare_parameter('num_sides', 4)

        self.pub = self.create_publisher(Float64MultiArray, CMD_TOPIC, 10)

        self.state = 'STRAIGHT'   # STRAIGHT -> TURN -> ... -> DONE
        self.side = 0
        self.phase_start = self.get_clock().now()
        self.create_timer(0.1, self.loop)   # 10 Hz
        self.get_logger().info('demo_square: chay VONG HO theo thoi gian (khong co odom).')

    def send(self, left, right):
        msg = Float64MultiArray()
        msg.data = [float(left), float(right)]
        self.pub.publish(msg)

    def loop(self):
        straight_vel = float(self.get_parameter('straight_vel').value)
        turn_vel = float(self.get_parameter('turn_vel').value)
        straight_time = float(self.get_parameter('straight_time').value)
        turn_time = float(self.get_parameter('turn_time').value)
        num_sides = int(self.get_parameter('num_sides').value)

        elapsed = (self.get_clock().now() - self.phase_start).nanoseconds * 1e-9

        if self.state == 'STRAIGHT':
            if elapsed >= straight_time:
                self.send(0.0, 0.0)
                self.get_logger().info(
                    f'Canh {self.side + 1}/{num_sides}: da di thang {straight_time}s -> quay.')
                self.state = 'TURN'
                self.phase_start = self.get_clock().now()
            else:
                self.send(straight_vel, straight_vel)   # 2 banh cung chieu

        elif self.state == 'TURN':
            if elapsed >= turn_time:
                self.send(0.0, 0.0)
                self.side += 1
                if self.side >= num_sides:
                    self.state = 'DONE'
                    self.get_logger().info('HOAN THANH LO TRINH (vong ho).')
                else:
                    self.state = 'STRAIGHT'
                    self.phase_start = self.get_clock().now()
            else:
                self.send(-turn_vel, turn_vel)   # quay trai: trai -, phai +

        elif self.state == 'DONE':
            self.send(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = DemoSquare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
