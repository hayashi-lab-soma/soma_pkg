cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(soma_mapping)

find_package(catkin REQUIRED
    roscpp
    sensor_msgs
    geometry_msgs
    pcl_ros
    pluginlib
    nodelet
    tf)

find_package(PCL 1.7 REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_proj src/pointcloud_projection.cpp)

target_link_libraries(pcd_proj ${PCL_LIBRARIES})