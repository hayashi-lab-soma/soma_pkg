cmake_minimum_required(VERSION 3.0.2)
project(soma_mse)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    dwa_local_planner
    tf2_ros
    tf
    nav_msgs
    geometry_msgs
    )

catkin_package(
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

add_executable(harvesting_behavior_node
    src/node/behavior_node.cpp
    src/StateBase/StateBase.cpp
    src/States/Stop.cpp
    src/States/MoveTo.cpp
    src/States/Home.cpp
    )

target_link_libraries(harvesting_behavior_node
    ${catkin_LIBRARIES}
    )

add_executable(sample_odom_subscriber_node
    src/node/sample_odom_subscribe_node.cpp
    )

target_link_libraries(sample_odom_subscriber_node
    ${catkin_LIBRARIES}
    )