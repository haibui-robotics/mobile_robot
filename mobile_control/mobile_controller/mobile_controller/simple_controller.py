#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from tf_transformations import quaternion_from_euler


class SimpleController(Node):

    def __init__(self):
        super().__init__("simple_controller")
        self.declare_parameter("wheel_radius", 0.1)
        self.declare_parameter("wheel_separation", 0.45)
        # [THÊM 2026-11-02] Tham số publish_tf:
        # - Đặt True khi chạy độc lập (không dùng EKF) để simple_controller tự phát TF odom -> base_footprint.
        # - Đặt False khi chạy kèm EKF (ekf_filter_node) để nhường quyền phát TF odom -> base_footprint cho EKF,
        #   tránh việc 2 node cùng phát 1 cây TF gây giật hình trên RViz2.
        self.declare_parameter("publish_tf", True)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value
        self.publish_tf_ = self.get_parameter("publish_tf").get_parameter_value().bool_value

        self.get_logger().info(f"Using wheel radius: {self.wheel_radius_}")
        self.get_logger().info(f"Using wheel separation: {self.wheel_separation_}")

        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        # [SỬA 2026-11-02] Cờ đánh dấu bản tin /joint_states đầu tiên để đồng bộ gốc tọa độ (0,0,0)
        # tránh việc xe bị nhảy vị trí trên RViz2 nếu ESP32 đã tích lũy sẵn xung encoder từ trước.
        self.first_joint_msg_ = True

        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.vel_sub_ = self.create_subscription(TwistStamped, "/mobile_controller/cmd_vel", self.velCallback, 10)
        self.joint_sub_ = self.create_subscription(JointState,"joint_states", self.jointCallback, 10)        
        self.odom_pub_ = self.create_publisher(Odometry, "mobile_controller/odom", 10)

        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2],
                                           [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])
        self.get_logger().info("The conversion matrix is %s" % self.speed_conversion_)

        # Fill the Odometry message with invariant parameters
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0
        # [THÊM 2026-11-02] Khai báo ma trận hiệp phương sai (Covariance) đường chéo cho Odometry
        # để bộ lọc EKF (robot_localization) có thể tính trọng số kết hợp với IMU chuẩn xác.
        self.odom_msg_.pose.covariance[0] = 0.001   # x
        self.odom_msg_.pose.covariance[7] = 0.001   # y
        self.odom_msg_.pose.covariance[35] = 0.05   # yaw (bánh xe dễ trượt khi xoay nên phương sai lớn hơn IMU)
        self.odom_msg_.twist.covariance[0] = 0.001  # vx
        self.odom_msg_.twist.covariance[7] = 0.001  # vy
        self.odom_msg_.twist.covariance[35] = 0.05  # vyaw

        # Fill the TF message
        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"

        self.prev_time_ = self.get_clock().now()

        # [SỬA 2026-11-02] Tạo timer 20Hz phát liên tục TF (odom -> base_footprint) ngay từ khi khởi động
        # để RViz2 luôn nhận diện được Fixed Frame 'odom' kể cả trong lúc đang chờ phần cứng khởi tạo.
        # [SỬA 2026-11-02] Chỉ tạo timer phát TF trực tiếp nếu publish_tf == True (khi không dùng EKF)
        if self.publish_tf_:
            self.tf_timer_ = self.create_timer(0.05, self.publishTfCallback)

    def publishTfCallback(self):
        # [SỬA 2026-11-02] Phát phép biến đổi TF odom -> base_footprint định kỳ
        q = quaternion_from_euler(0, 0, self.theta_)
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.br_.sendTransform(self.transform_stamped_)


    def velCallback(self, msg):
        # Implements the differential kinematic model
        # Given v and w, calculate the velocities of the wheels
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed) 

        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]

        self.wheel_cmd_pub_.publish(wheel_speed_msg)

    
    def jointCallback(self, msg):
        # Implements the inverse differential kinematic model
        # Given the position of the wheels, calculates their velocities
        # then calculates the velocity of the robot wrt the robot frame
        # and then converts it in the global frame and publishes the TF

        # [FIX] Tim dung index cua 2 banh theo ten (khong gia dinh position[0]/[1])
        try:
            left_idx = msg.name.index("left_wheel_joint")
            right_idx = msg.name.index("right_wheel_joint")
        except ValueError:
            self.get_logger().warning(
                "joint_states thieu 'left_wheel_joint' hoac 'right_wheel_joint'",
                throttle_duration_sec=2.0)
            return

        # [SỬA 2026-11-02] Ở bản tin đầu tiên, lưu lại vị trí góc bánh xe làm mốc tham chiếu ban đầu
        if self.first_joint_msg_:
            self.left_wheel_prev_pos_ = msg.position[left_idx]
            self.right_wheel_prev_pos_ = msg.position[right_idx]
            self.prev_time_ = Time.from_msg(msg.header.stamp)
            self.first_joint_msg_ = False
            return

        dp_left = msg.position[left_idx] - self.left_wheel_prev_pos_
        dp_right = msg.position[right_idx] - self.right_wheel_prev_pos_
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        # Actualize the prev pose for the next itheration
        self.left_wheel_prev_pos_ = msg.position[left_idx]
        self.right_wheel_prev_pos_ = msg.position[right_idx]
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        # [FIX] Bo qua neu dt <= 0 (tranh chia cho 0 / van toc sai)
        if dt.nanoseconds <= 0:
            return

        # Calculate the rotational speed of each wheel
        fi_left = dp_left / (dt.nanoseconds / S_TO_NS)
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS)

        # Calculate the linear and angular velocity
        linear = (self.wheel_radius_ * fi_right + self.wheel_radius_ * fi_left) / 2
        angular = (self.wheel_radius_ * fi_right - self.wheel_radius_ * fi_left) / self.wheel_separation_

        # Calculate the position increment
        d_s = (self.wheel_radius_ * dp_right + self.wheel_radius_ * dp_left) / 2
        d_theta = (self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left) / self.wheel_separation_
        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)
        
        # Compose and publish the odom message
        q = quaternion_from_euler(0, 0, self.theta_)
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular
        self.odom_pub_.publish(self.odom_msg_)

        # TF
        # [SỬA 2026-11-02] Chỉ phát TF odom -> base_footprint từ simple_controller khi publish_tf_ == True.
        # Khi dùng EKF (publish_tf_ == False), ekf_filter_node sẽ đảm nhận phát TF odom -> base_footprint đã lọc.
        if self.publish_tf_:
            self.transform_stamped_.transform.translation.x = self.x_
            self.transform_stamped_.transform.translation.y = self.y_
            self.transform_stamped_.transform.rotation.x = q[0]
            self.transform_stamped_.transform.rotation.y = q[1]
            self.transform_stamped_.transform.rotation.z = q[2]
            self.transform_stamped_.transform.rotation.w = q[3]
            self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
            self.br_.sendTransform(self.transform_stamped_)


def main():
    rclpy.init()

    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    
    simple_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()