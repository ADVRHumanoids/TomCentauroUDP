cmake_minimum_required(VERSION 3.16)
project(tom_centauro)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(catkin REQUIRED)
find_package(roscpp REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

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

# test ros velocity client
add_executable(${PROJECT_NAME}_client_velocity_ros
    src/udp_client_velocity_ros.cpp)

target_include_directories(${PROJECT_NAME}_client_velocity_ros
    PRIVATE ${roscpp_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}_client_velocity_ros
    PRIVATE ${roscpp_LIBRARIES} udp_server Eigen3::Eigen)

# test dummy tom client
add_executable(${PROJECT_NAME}_dummy_client
    src/udp_dummy_client.cpp)

target_include_directories(${PROJECT_NAME}_dummy_client
    PRIVATE ${roscpp_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}_dummy_client
    PRIVATE ${roscpp_LIBRARIES} udp_server Eigen3::Eigen)


# xbot2 plugin
find_package(xbot2 QUIET)

if(${xbot2_FOUND})

    find_package(cartesian_interface REQUIRED)

    add_xbot2_plugin(${PROJECT_NAME}_udp_server
        src/udp_server_plugin.cpp)

    add_xbot2_plugin(${PROJECT_NAME}_ik
        src/ik_plugin.cpp)

    target_link_libraries(${PROJECT_NAME}_ik PRIVATE cartesian_interface::CartesianInterface )

    target_link_libraries(${PROJECT_NAME}_udp_server PRIVATE udp_server)

    install(TARGETS ${PROJECT_NAME}_udp_server ${PROJECT_NAME}_ik
        DESTINATION lib)

endif()

install(TARGETS
    ${PROJECT_NAME}_server_ros
    ${PROJECT_NAME}_client_ros
    ${PROJECT_NAME}_client_velocity_ros
    DESTINATION bin)
