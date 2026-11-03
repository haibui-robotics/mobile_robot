cmake_minimum_required(VERSION 3.8)
project(mobile_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(mobile_msgs REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  imu_republisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME imu_republisher
)
install(PROGRAMS
  kalman_filter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME kalman_filter
)
install(PROGRAMS
  odometry_motion_model.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME odometry_motion_model
)

ament_package()
