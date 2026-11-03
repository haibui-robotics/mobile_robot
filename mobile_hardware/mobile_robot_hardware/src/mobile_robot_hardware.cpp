#include "mobile_robot_hardware/mobile_robot_hardware.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace mobile_robot_hardware
{

MobileRobotInterface::MobileRobotInterface()
{
}

MobileRobotInterface::~MobileRobotInterface()
{
  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("MobileRobotInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}

CallbackReturn MobileRobotInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  try
  {
    port_ = info_.hardware_parameters.at("serial_port");
  }
  catch (const std::out_of_range & e)
  {
    RCLCPP_WARN(rclcpp::get_logger("MobileRobotInterface"),
                "No 'serial_port' parameter provided! Defaulting to /dev/ttyUSB0");
    port_ = "/dev/ttyUSB0";
  }

  try
  {
    encoder_cpr_ = std::stod(info_.hardware_parameters.at("encoder_cpr"));
  }
  catch (const std::out_of_range & e)
  {
    encoder_cpr_ = 1234.3;
  }

  try
  {
    max_wheel_rad_s_ = std::stod(info_.hardware_parameters.at("max_wheel_rad_s"));
  }
  catch (...)
  {
    max_wheel_rad_s_ = 10.0;
  }

  try
  {
    use_pwm_ = (info_.hardware_parameters.at("use_pwm") == "true");
  }
  catch (...)
  {
    use_pwm_ = false;
  }

  velocity_commands_.resize(info_.joints.size(), 0.0);
  position_states_.resize(info_.joints.size(), 0.0);
  velocity_states_.resize(info_.joints.size(), 0.0);
  last_run_ = rclcpp::Clock().now();
  first_read_ = true;

  RCLCPP_INFO(rclcpp::get_logger("MobileRobotInterface"),
              "Initialized with port: %s, CPR: %.1f, use_pwm: %s",
              port_.c_str(), encoder_cpr_, use_pwm_ ? "true" : "false");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MobileRobotInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MobileRobotInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn MobileRobotInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("MobileRobotInterface"), "Starting robot hardware on port %s ...", port_.c_str());

  velocity_commands_.assign(info_.joints.size(), 0.0);
  position_states_.assign(info_.joints.size(), 0.0);
  velocity_states_.assign(info_.joints.size(), 0.0);
  first_read_ = true;

  try
  {
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    // [SỬA 2026-11-02] Tăng thời gian chờ từ 1500ms lên 2200ms để ESP32 kịp hoàn tất
    // quá trình tự động lấy 200 mẫu hiệu chuẩn tĩnh (Calibration) cho MPU6050 khi mới mở cổng Serial.
    std::this_thread::sleep_for(std::chrono::milliseconds(2200));
    // [SỬA 2026-11-02] Xóa bộ đệm Serial sau khi đợi ESP32 khởi động
    // để loại bỏ dữ liệu rác lúc boot và tránh tồn đọng hàng chục dòng cũ làm nghẽn hàm read().
    arduino_.FlushIOBuffers();
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("MobileRobotInterface"),
                        "Something went wrong while opening serial port " << port_);
    return CallbackReturn::FAILURE;
  }

  // [THÊM 2026-11-02] Khởi tạo publisher /imu/out sử dụng node tích hợp sẵn của HardwareComponentInterface
  if (get_node() && !imu_pub_)
  {
    imu_pub_ = get_node()->create_publisher<sensor_msgs::msg::Imu>("/imu/out", 10);
    RCLCPP_INFO(rclcpp::get_logger("MobileRobotInterface"),
                "IMU Publisher initialized on topic /imu/out (frame_id: imu_link)");
  }

  RCLCPP_INFO(rclcpp::get_logger("MobileRobotInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MobileRobotInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("MobileRobotInterface"), "Stopping robot hardware ...");

  if (arduino_.IsOpen())
  {
    try
    {
      if (use_pwm_)
      {
        arduino_.Write("0 0\n");
      }
      else
      {
        arduino_.Write("L0.00 R0.00\n");
      }
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("MobileRobotInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("MobileRobotInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MobileRobotInterface::read(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  if (!arduino_.IsOpen())
  {
    return hardware_interface::return_type::OK;
  }

  // Firmware ESP32 gui: "L<left_count> R<right_count>\n" (so xung encoder tich luy)
  // [SỬA 2026-11-02] Giới hạn số dòng đọc tối đa mỗi chu kỳ (max_lines = 10) và đặt timeout 2ms cho ReadLine
  // (mặc định msTimeout=0 của LibSerial là đợi vô hạn nếu chuỗi chưa có ký tự '\n', dễ làm treo controller_manager).
  int lines_read = 0;
  while (arduino_.IsDataAvailable() && lines_read < 10)
  {
    lines_read++;
    const double dt = std::max((rclcpp::Clock().now() - last_run_).seconds(), 1e-6);

    std::string message;
    try
    {
      // [SỬA 2026-11-02] Tăng timeout từ 2ms lên 10ms vì chuỗi có thêm dữ liệu IMU dài ~72 ký tự
      // (ở tốc độ 115200 baud cần ~6.3ms để truyền hết 1 dòng, nếu để 2ms sẽ dễ bị cắt cụt phần GZ và YAW ở cuối dòng).
      arduino_.ReadLine(message, '\n', 10);
    }
    catch (...)
    {
      break;
    }

    std::stringstream ss(message);
    std::string token;

    // [THÊM 2026-11-02] Các biến tạm để gom dữ liệu IMU từ dòng Serial:
    // Định dạng từ ESP32: "L<cur_L> R<cur_R> AX<ax> AY<ay> AZ<az> GX<gx> GY<gy> GZ<gz> YAW<yaw>"
    bool has_imu_data = false;
    double ax = 0.0, ay = 0.0, az = 9.80665;
    double gx = 0.0, gy = 0.0, gz = 0.0;
    double yaw = 0.0;

    while (ss >> token)
    {
      if (token.size() < 2)
      {
        continue;
      }

      // [THÊM 2026-11-02] Phân tích các trường dữ liệu IMU (AX, AY, AZ, GX, GY, GZ, YAW)
      try
      {
        if (token.rfind("AX", 0) == 0) { ax = std::stod(token.substr(2)); has_imu_data = true; continue; }
        if (token.rfind("AY", 0) == 0) { ay = std::stod(token.substr(2)); has_imu_data = true; continue; }
        if (token.rfind("AZ", 0) == 0) { az = std::stod(token.substr(2)); has_imu_data = true; continue; }
        if (token.rfind("GX", 0) == 0) { gx = std::stod(token.substr(2)); has_imu_data = true; continue; }
        if (token.rfind("GY", 0) == 0) { gy = std::stod(token.substr(2)); has_imu_data = true; continue; }
        if (token.rfind("GZ", 0) == 0) { gz = std::stod(token.substr(2)); has_imu_data = true; continue; }
        if (token.rfind("YAW", 0) == 0) { yaw = std::stod(token.substr(3)); has_imu_data = true; continue; }
      }
      catch (...)
      {
        continue;
      }

      if (token[0] != 'L' && token[0] != 'R')
      {
        continue;
      }

      // Joint order theo xacro: index 0 = right_wheel_joint, index 1 = left_wheel_joint
      const size_t wheel_idx = (token[0] == 'L') ? 1 : 0;
      try
      {
        const double count = std::stod(token.substr(1));
        const double pos = (count / encoder_cpr_) * (2.0 * M_PI);

        if (first_read_)
        {
          velocity_states_.at(wheel_idx) = 0.0;
        }
        else
        {
          velocity_states_.at(wheel_idx) = (pos - position_states_.at(wheel_idx)) / dt;
        }
        position_states_.at(wheel_idx) = pos;
      }
      catch (...)
      {
        continue;
      }
    }

    // [THÊM 2026-11-02] Nếu dòng Serial có dữ liệu MPU6050 thì đóng gói sensor_msgs/msg/Imu và publish lên /imu/out
    if (has_imu_data && imu_pub_)
    {
      sensor_msgs::msg::Imu imu_msg;
      imu_msg.header.stamp = get_node() ? get_node()->now() : rclcpp::Clock().now();
      imu_msg.header.frame_id = "imu_link";

      // [SỬA 2026-11-02] Tính thêm góc nghiêng Roll, Pitch từ gia tốc kế (ax, ay, az) kết hợp với góc hướng Yaw
      // để chuyển sang Quaternion 3D đầy đủ (qx, qy, qz, qw), giúp quan sát được cả khi nghiêng xe lẫn xoay xe.
      const double roll = std::atan2(ay, az);
      const double pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
      const double cy = std::cos(yaw * 0.5);
      const double sy = std::sin(yaw * 0.5);
      const double cp = std::cos(pitch * 0.5);
      const double sp = std::sin(pitch * 0.5);
      const double cr = std::cos(roll * 0.5);
      const double sr = std::sin(roll * 0.5);

      imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
      imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
      imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;
      imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
      // Ma trận hiệp phương sai (Covariance) cho Orientation (cần thiết cho EKF / robot_localization)
      imu_msg.orientation_covariance[0] = 0.01;
      imu_msg.orientation_covariance[4] = 0.01;
      imu_msg.orientation_covariance[8] = 0.005;

      // Vận tốc góc (rad/s) - kalman_filter.py và EKF sử dụng trực tiếp angular_velocity.z
      imu_msg.angular_velocity.x = gx;
      imu_msg.angular_velocity.y = gy;
      imu_msg.angular_velocity.z = gz;
      imu_msg.angular_velocity_covariance[0] = 0.001;
      imu_msg.angular_velocity_covariance[4] = 0.001;
      imu_msg.angular_velocity_covariance[8] = 0.001;

      // Gia tốc tuyến tính (m/s^2)
      imu_msg.linear_acceleration.x = ax;
      imu_msg.linear_acceleration.y = ay;
      imu_msg.linear_acceleration.z = az;
      imu_msg.linear_acceleration_covariance[0] = 0.01;
      imu_msg.linear_acceleration_covariance[4] = 0.01;
      imu_msg.linear_acceleration_covariance[8] = 0.01;

      imu_pub_->publish(imu_msg);
    }

    first_read_ = false;
    last_run_ = rclcpp::Clock().now();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MobileRobotInterface::write(const rclcpp::Time &,
                                                           const rclcpp::Duration &)
{
  if (!arduino_.IsOpen())
  {
    return hardware_interface::return_type::ERROR;
  }

  std::stringstream message_stream;

  if (use_pwm_)
  {
    int pwm_left = std::clamp(static_cast<int>(velocity_commands_.at(1) / max_wheel_rad_s_ * 255.0), -255, 255);
    int pwm_right = std::clamp(static_cast<int>(velocity_commands_.at(0) / max_wheel_rad_s_ * 255.0), -255, 255);
    message_stream << pwm_left << " " << pwm_right << "\n";
  }
  else
  {
    message_stream << std::fixed << std::setprecision(2)
                   << "L" << velocity_commands_.at(1)
                   << " R" << velocity_commands_.at(0)
                   << "\n";
  }

  try
  {
    arduino_.Write(message_stream.str());
  }
  catch (...)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MobileRobotInterface"),
                 "Failed to send command to port %s", port_.c_str());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace mobile_robot_hardware

PLUGINLIB_EXPORT_CLASS(mobile_robot_hardware::MobileRobotInterface, hardware_interface::SystemInterface)
