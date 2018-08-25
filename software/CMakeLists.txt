cmake_minimum_required(VERSION 3.10)
project(robocar)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)
file(GLOB SOURCES "src/*.cpp")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")

add_executable(robocar ${SOURCES})