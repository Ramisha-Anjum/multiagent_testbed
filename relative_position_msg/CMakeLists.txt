cmake_minimum_required(VERSION 3.8)
project(relative_position_msg)

# Core
find_package(ament_cmake REQUIRED)

# Generators for IDL/msg
find_package(rosidl_default_generators REQUIRED)

# >>> VERY IMPORTANT: find interface dependencies BEFORE rosidl_generate_interfaces <<<
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate your interfaces
# Ensure these files actually exist under msg/
#   msg/RelativeMeasurement.msg
#   msg/RelativeNeighbors.msg
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RelativeMeasurement.msg"
  "msg/RelativeNeighbors.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Let downstream packages find/use these generated interfaces
ament_export_dependencies(rosidl_default_runtime)

ament_package()
