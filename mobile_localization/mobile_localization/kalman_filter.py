#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class KalmanFilter(Node):

    def __init__(self):
        super().__init__("kalman_filter")
        self.odom_sub_ = self.create_subscription(Odometry, "mobile_controller/odom_noisy", self.odomCallback, 10)
        self.imu_sub_ = self.create_subscription(Imu, "imu/out", self.imuCallback, 10)
        self.odom_pub_ = self.create_publisher(Odometry, "mobile_controller/odom_kalman", 10)
        
       
        self.mean_ = 0.0   # Vận tốc góc trung bình ước lượng ban đầu
        self.variance_ = 1000.0  # Phương sai ban đầu rất lớn (Robot chưa biết chắc tốc độ quay)


        self.motion_variance_ = 4.0  # Nhiễu của Odometry bánh xe (khá lớn)
        self.measurement_variance_ = 0.5  # Nhiễu của IMU (nhỏ hơn => tin tưởng IMU hơn)

      
        self.imu_angular_z_ = 0.0

        self.is_first_odom_ = True
        self.last_angular_z_ = 0.0
        self.motion_ = 0.0

        # Publish the filtered odometry message
        self.kalman_odom_ = Odometry()


    def odomCallback(self, odom):
        self.kalman_odom_ = odom
# Xử lý riêng cho lần nhận dữ liệu Odometry đầu tiên
        if self.is_first_odom_:
            self.last_angular_z_ = odom.twist.twist.angular.z
            self.is_first_odom_ = False
            self.mean_ = odom.twist.twist.angular.z
            return
        # Tính lượng thay đổi vận tốc góc (u = motion) từ bước trước đến bước này
        self.motion_ = odom.twist.twist.angular.z - self.last_angular_z_

        # Thực hiện 2 bước của Kalman Filter
        self.statePrediction()     # 1. Predict
        self.measurementUpdate()   # 2. Update

        # Cập nhật cho bước lặp kế tiếp
        self.last_angular_z_ = odom.twist.twist.angular.z

        # Ghi đè giá trị angular.z đã qua lọc Kalman vào tin nhắn Odom và Publish
        self.kalman_odom_.twist.twist.angular.z = self.mean_
        self.odom_pub_.publish(self.kalman_odom_)


    def imuCallback(self, imu):
  
        self.imu_angular_z_ = imu.angular_velocity.z  #luu lai gia tri do duoc vao bien


    def measurementUpdate(self):
        self.mean_ = (self.measurement_variance_ * self.mean_ + self.variance_ * self.imu_angular_z_) \
                   / (self.variance_ + self.measurement_variance_)
                     
        self.variance_ = (self.variance_ * self.measurement_variance_) \
                       / (self.variance_ + self.measurement_variance_)


    def statePrediction(self):
        self.mean_ = self.mean_ + self.motion_
        self.variance_ = self.variance_ + self.motion_variance_


def main():
    rclpy.init()
    kalman_filter = KalmanFilter()
    rclpy.spin(kalman_filter)
    kalman_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()