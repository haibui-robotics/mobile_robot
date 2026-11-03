cmake_minimum_required(VERSION 3.8)
project(mobile_robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
# [THÊM 2026-11-02] Tìm gói sensor_msgs để sử dụng kiểu bản tin sensor_msgs::msg::Imu
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# Tạo thư viện C++ shared object
add_library(${PROJECT_NAME} SHARED
  src/mobile_robot_hardware.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  # [THÊM 2026-11-02] Liên kết thư viện sensor_msgs cho publisher /imu/out
  sensor_msgs
)

# [FIX] Lien ket libserial (thu vien cong serial)
target_include_directories(${PROJECT_NAME} PRIVATE ${LIBSERIAL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${LIBSERIAL_LIBRARIES})

# Khai báo Plugin với pluginlib
pluginlib_export_plugin_description_file(hardware_interface mobile_robot_hardware.xml)

# Export thư viện
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES mobile_robot_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()