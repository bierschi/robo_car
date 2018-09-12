cmake_minimum_required(VERSION 3.7)

project(client)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)

file(GLOB SOURCES "main_clientsocket.cpp" "src/*.cpp")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")

add_executable(client ${SOURCES})

