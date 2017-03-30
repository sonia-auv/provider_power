# \file     CMakeLists.txt
# \author   Antoine Dozois <dozois.a@gmail.com>
# \date     10/16/2015
# \copyright    2015 Club SONIA AUV, ETS. All rights reserved.
# Use of this source code is governed by the MIT license that can be
# found in the LICENSE file.

#===============================================================================
# G L O B A L   P R O J E C T   C O N F I G U R A T I O N

cmake_minimum_required(VERSION 2.8.3)
project(provider_power)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING
            "Choose the type of build: None Debug Release RelWithDebInfo MinSizeRel."
            FORCE)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -O3")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package( catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
        lib_atlas
        interface_rs485
        )


file(GLOB srv_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/srv/"
        "${CMAKE_CURRENT_SOURCE_DIR}/srv/*.srv")
add_service_files(DIRECTORY srv FILES ${srv_files})
add_message_files(DIRECTORY msg FILES powerMsg.msg)

generate_messages(DEPENDENCIES std_msgs )

# Declare this project as a catkin package and export the necessary build flags
catkin_package(
        INCLUDE_DIRS src
        LIBRARIES
        CATKIN_DEPENDS
        roscpp
        std_msgs
        message_runtime
        lib_atlas
        interface_rs485
)



#===============================================================================
# I N C L U D E   F I L E S

set(provider_power_INCLUDE_DIRS "src")
set(provider_power_SRC_DIR "src/provider_power")

# Include all the directories where source can be found.
include_directories(
        ${provider_power_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

#==========================================================================
# C R E A T E   E X E C U T A B L E

set( TARGET_NAME provider_power_node )
add_executable( ${TARGET_NAME}
        ${provider_power_SRC_DIR}/main.cc
        ${provider_power_SRC_DIR}/provider_power_node.cc)
target_link_libraries( ${TARGET_NAME} ${catkin_LIBRARIES})
set_target_properties( ${TARGET_NAME} PROPERTIES LINKER_LANGUAGE CXX )
add_dependencies(${PROJECT_NAME}_node
        ${PROJECT_NAME}_generate_messages_cpp)