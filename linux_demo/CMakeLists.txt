# CMakeLists text file of the project
# RFAL code release version 1.0

cmake_minimum_required(VERSION 3.1.0)

project(rfal_demo)

# RFAL library variant to link the executable with
set(RFAL_VARIANT "st25r3911" CACHE STRING "Select the RFAL library")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -ggdb")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,stack-size=1000000")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
message(${CMAKE_CXX_COMPILER_ID})
message(${CMAKE_EXE_LINKER_FLAGS})

# add sub folder to build as per CMakelists.txt in the sub-folder
add_subdirectory(spirit_application)
