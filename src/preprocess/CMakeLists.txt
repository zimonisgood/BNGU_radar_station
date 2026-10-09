cmake_minimum_required(VERSION 3.5...3.13)
project(roi)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

add_executable(roi_node src/roi_node.cpp)
target_include_directories(roi_node SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_options(roi_node PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(roi_node ${PCL_LIBRARIES})
ament_target_dependencies(roi_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

install(TARGETS roi_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
