cmake_minimum_required(VERSION 3.8)
project(mobile_controller)

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
  mobile_controller/teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop_keyboard
)
install(PROGRAMS
  mobile_controller/demo_square.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME demo_square
)

install(PROGRAMS
  mobile_controller/wheel_test.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wheel_test
)

install(PROGRAMS
  mobile_controller/test_jointgroupvelocity.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_jointgroupvelocity
)
install(PROGRAMS
  mobile_controller/test_tf_static_dynamic.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_tf_static_dynamic
)
install(PROGRAMS
  mobile_controller/simple_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME simple_controller
)
install(PROGRAMS
  mobile_controller/teleop_twiststamped.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop_twiststamped
)
install(PROGRAMS
  mobile_controller/noisy_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME noisy_controller
)

ament_package()
