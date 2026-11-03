#ifndef MOBILE_ROBOT_HARDWARE__MOBILE_ROBOT_HARDWARE_HPP_
#define MOBILE_ROBOT_HARDWARE__MOBILE_ROBOT_HARDWARE_HPP_

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
// [THÊM 2026-11-02] Thư viện bản tin IMU chuẩn của ROS 2 để publish dữ liệu MPU6050 lên topic /imu/out
#include <sensor_msgs/msg/imu.hpp>

#include <memory>
#include <string>
#include <vector>

namespace mobile_robot_hardware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MobileRobotInterface : public hardware_interface::SystemInterface
{
public:
  MobileRobotInterface();
  virtual ~MobileRobotInterface();

  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  LibSerial::SerialPort arduino_;
  std::string port_ = "/dev/ttyUSB0";
  double encoder_cpr_ = 1234.3;
  double max_wheel_rad_s_ = 1.0;
  bool use_pwm_ = false;

  bool first_read_ = true;
  std::vector<double> velocity_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  rclcpp::Time last_run_;

  // [THÊM 2026-11-02] Publisher phát dữ liệu cảm biến MPU6050 đã hiệu chuẩn lên topic /imu/out
  // (Khớp hoàn toàn với cấu hình của kalman_filter.py, imu_republisher.py, ekf.yaml và Gazebo)
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

}  // namespace mobile_robot_hardware

#endif  // MOBILE_ROBOT_HARDWARE__MOBILE_ROBOT_HARDWARE_HPP_
