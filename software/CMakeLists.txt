cmake_minimum_required(VERSION 3.7)
project(robocar)

set(CMAKE_CXX_STANDARD 11)

#find_package(OpenCV REQUIRED)
#include_directories( ${OpenCV_INCLUDE_DIRS})
include_directories(include)
file(GLOB SOURCES "src/*.cpp")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")

add_executable(robocar ${SOURCES} include/client.h src/client.cpp)
#target_link_libraries( robocar ${OpenCV_LIBS})
