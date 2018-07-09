# CMake minimum version and policies
cmake_minimum_required(VERSION 3.0.0)

# centauro UDP receiver
project(TomCentauroUDP)

# Add cmake dir and include custom macros
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroOptionalBuild)
include(MacroInstallLibrary)


# C++11 
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Enable Xenomai flag and if enabled find required packages
set(ENABLE_XENO CACHE BOOL "ON")
if ( ${ENABLE_XENO} )
   find_package(Xenomai)
   find_package(RTnet)
endif()

find_package(XBotInterface REQUIRED)

# include 
include_directories(include ${XBotInterface_INCLUDE_DIRS}) 

                                   
########################################################################


# set Xenomain flags if Xenomai is founded
if ( Xenomai_FOUND )
    # exe
    add_executable(tom_centauro_udp_receiver
        src/centauro_udp_receiver.cpp
        src/rt_ipc.cpp
    )
    set_xeno_flags(tom_centauro_udp_receiver)
else ()
    # exe
    add_executable(tom_centauro_udp_receiver
        src/centauro_udp_receiver.cpp
    )
endif()

target_link_libraries(tom_centauro_udp_receiver ${XBotInterface_LIBRARIES})
library_install(tom_centauro_udp_receiver 1 0 0)


########################################################################


