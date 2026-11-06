cmake_minimum_required(VERSION 3.5...3.13)
project(registration)

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
set(CMAKE_POLICY_DEFAULT_CMP0074 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)

# 配准流水线各节点(每个功能一个独立可执行文件)
set(registration_nodes
    icp_node                # ICP 配准到静态参考点云
    background_filter_node  # 先验占据栅格背景过滤
)

foreach(node ${registration_nodes})
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

install(TARGETS ${registration_nodes}
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config mesh
    DESTINATION share/${PROJECT_NAME})

ament_package()