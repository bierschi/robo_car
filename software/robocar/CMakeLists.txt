cmake_minimum_required(VERSION 3.7)
project(robocar)

set(CMAKE_CXX_STANDARD 11)

find_package(Threads REQUIRED)
find_package(WiringPi REQUIRED)
#find_package(OpenCV REQUIRED)

#include_directories( ${OpenCV_INCLUDE_DIRS})
include_directories(${WIRINGPI_INCLUDE_DIRS})
include_directories(include)

file(GLOB SOURCES "robocar_main.cpp" "src/comm/*.cpp" "src/sensors/*cpp")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")


add_executable(robocar ${SOURCES})
target_link_libraries(robocar ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(robocar ${WIRINGPI_LIBRARIES})
#target_link_libraries( robocar ${OpenCV_LIBS})

