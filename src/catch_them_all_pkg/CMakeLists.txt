cmake_minimum_required(VERSION 3.8)
project(catch_them_all_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(catch_them_all_interface REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(turtle_spawn src/turtle_spawn.cpp)
ament_target_dependencies(turtle_spawn
  rclcpp
  catch_them_all_interface
  turtlesim
)

add_executable(turtle_chase src/turtle_chase.cpp)
ament_target_dependencies(turtle_chase
  rclcpp
  catch_them_all_interface
  turtlesim
  geometry_msgs
)
install(TARGETS
    turtle_spawn
    turtle_chase
    DESTINATION lib/${PROJECT_NAME})
    
ament_export_dependencies(catch_them_all_interface) 
ament_package()
