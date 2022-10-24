cmake_minimum_required(VERSION 3.19)
project(tom_centauro_udp)

find_package(roscpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

# udp server and client library
add_library(udp_server STATIC
    src/udp_server.cpp)

# test ros server
add_executable(udp_server_ros
    src/udp_server_ros.cpp)

target_include_directories(udp_server_ros
    PRIVATE ${roscpp_INCLUDE_DIRS})

target_link_libraries(udp_server_ros
    PRIVATE ${roscpp_LIBRARIES} udp_server Eigen3::Eigen)

# test ros client
add_executable(udp_client_ros
    src/udp_client_ros.cpp)

target_include_directories(udp_client_ros
    PRIVATE ${roscpp_INCLUDE_DIRS})

target_link_libraries(udp_client_ros
    PRIVATE ${roscpp_LIBRARIES} udp_server Eigen3::Eigen)

# xbot2 plugin
find_package(xbot2 QUIET)

if(${xbot2_FOUND})
    add_xbot2_plugin(udp_server_plugin
        src/udp_server_plugin.cpp)
endif()
