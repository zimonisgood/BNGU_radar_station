cmake_minimum_required(VERSION 3.5...3.13)
project(preprocess)

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

# 预处理流水线各节点(多节点架构:每个功能一个独立可执行文件)
set(preprocess_nodes
  roi_node                 # ROI 立方体裁剪
  invalid_removal_node     # 无效点(NaN/Inf)去除
  statistical_outlier_node # 统计离群点去除
)

foreach(node ${preprocess_nodes})
  add_executable(${node} src/${node}.cpp)
  target_include_directories(${node} SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
  target_compile_options(${node} PRIVATE ${PCL_DEFINITIONS})
  target_link_libraries(${node} ${PCL_LIBRARIES})
  ament_target_dependencies(${node}
    rclcpp
    sensor_msgs
    pcl_conversions
  )
endforeach()

install(TARGETS ${preprocess_nodes}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
