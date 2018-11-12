cmake_minimum_required(VERSION 3.3)

project(client)

set(CMAKE_CXX_STANDARD 11)

find_package(Threads REQUIRED)

include_directories(include)

file(GLOB SOURCES "main.cpp" "src/*.cpp")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")


add_executable(client ${SOURCES})

target_link_libraries(client ${CMAKE_THREAD_LIBS_INIT})
