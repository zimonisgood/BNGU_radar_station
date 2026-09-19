cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(roi)

find_package(PCL 1.2 REQUIRED COMPONENTS common io)

add_executable (roi_demo roi.cpp)
target_compile_features(roi_demo PRIVATE cxx_std_17)
target_include_directories(roi_demo SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_options(roi_demo PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(roi_demo PRIVATE ${PCL_LIBRARIES})
