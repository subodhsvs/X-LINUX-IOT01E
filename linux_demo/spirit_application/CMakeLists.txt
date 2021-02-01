# Bring headers into the project
include_directories(
    ../../common/firmware/STM/utils/Inc
    ../platform/Inc
    ../platform/Inc/spirit1
    ./Inc)

file(GLOB SOURCE1 "Src/*.c")
file(GLOB SOURCE2 "../platform/Src/*.c")
set(SOURCES ${SOURCE1} ${SOURCE2})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -ggdb")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,stack-size=1000000")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Set the executable name
add_executable(spirit_application_binary ${SOURCES})

# Set the library to link with
target_link_libraries(spirit_application_binary PRIVATE m Threads::Threads)
