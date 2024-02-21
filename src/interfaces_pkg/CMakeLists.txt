cmake_minimum_required(VERSION 3.8)
project(interfaces_pkg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoundingBox2D.msg"
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/Mask.msg"
  "msg/Motor.msg"
  "msg/Point2D.msg"
  
  "msg/LaneInfo.msg"
  
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
