cmake_minimum_required(VERSION 3.5)
project(magiclaw_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Motor.msg"
  "msg/Claw.msg"
  "msg/Finger.msg"
  "msg/Phone.msg"
  "msg/MagiClaw.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()