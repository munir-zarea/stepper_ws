cmake_minimum_required(VERSION 3.8)
project(stepper_joy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# -------- Stepper node --------
add_executable(stepper_joy_node src/stepper_joy_node.cpp)
ament_target_dependencies(stepper_joy_node rclcpp sensor_msgs)

# -------- Servo serial node (name MUST match launch) --------
add_executable(joystick_servo6_serial_node src/joystick_servo6_serial_node.cpp)
ament_target_dependencies(joystick_servo6_serial_node rclcpp sensor_msgs)

install(TARGETS
  stepper_joy_node
  joystick_servo6_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
