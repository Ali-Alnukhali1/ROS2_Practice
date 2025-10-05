cmake_minimum_required(VERSION 3.8)
project(catch_them_all_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate message code
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TurtleInfo.msg"
)


ament_package()
