#########
# Setup #
#########

# Specify the minimum version for CMake
cmake_minimum_required(VERSION 2.8)

# Message
message(STATUS "CENG 5270 hw1")
message(STATUS ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS ${PROJECT_SOURCE_DIR})

# Find includes in the build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Project's name
project(par)

# Set the output folder where your program will be created
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(CMAKE_CXX_STANDARD 14)

set(PATH_SRC ${CMAKE_CURRENT_SOURCE_DIR})

###################
# Warnings/Errors #
###################

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Werror=return-type")

######################
# Check Dependencies #
######################
# uncommment this line if you are using self-compiled boost lib
# set(Boost_USE_STATIC_LIBS ON)
find_package(Boost COMPONENTS system filesystem program_options REQUIRED)
# find_package(Threads)

###############
# Source Code #
###############
file(GLOB_RECURSE SRC_FILES
    ${PATH_SRC}/*.cpp
    ${PATH_SRC}/*.cc
    ${PATH_SRC}/*.c)

###########
# Targets #
###########

add_executable(par ${SRC_FILES})

#######################
# Include Directories #
#######################

include_directories(${PATH_SRC})

#################
# Linker Flags  #
#################

# TODO: set static under release mode only
set_target_properties(par PROPERTIES LINK_FLAGS "-static -Wl,--whole-archive -rdynamic -lpthread -Wl,--no-whole-archive")

# Boost
target_include_directories(par PUBLIC ${Boost_INCLUDE_DIR})
target_link_libraries(par ${Boost_LIBRARIES})
