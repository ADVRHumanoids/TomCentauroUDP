cmake_minimum_required(VERSION 3.16)
project(tom_centauro_udp)

find_package(roscpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

# udp server and client library
add_library(udp_server STATIC
    src/udp_server.cpp)

# test ros server
add_executable(${PROJECT_NAME}_server_ros
    src/udp_server_ros.cpp)

target_include_directories(${PROJECT_NAME}_server_ros
    PRIVATE ${roscpp_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}_server_ros
    PRIVATE ${roscpp_LIBRARIES} udp_server Eigen3::Eigen)

# test ros client
add_executable(${PROJECT_NAME}_client_ros
    src/udp_client_ros.cpp)

target_include_directories(${PROJECT_NAME}_client_ros
    PRIVATE ${roscpp_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}_client_ros
    PRIVATE ${roscpp_LIBRARIES} udp_server Eigen3::Eigen)

# xbot2 plugin
find_package(xbot2 QUIET)

if(${xbot2_FOUND})
    add_xbot2_plugin(${PROJECT_NAME}_server_plugin
        src/udp_server_plugin.cpp)
endif()

install(TARGETS ${PROJECT_NAME}_server_ros ${PROJECT_NAME}_client_ros
    DESTINATION bin)
