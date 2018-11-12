cmake_minimum_required(VERSION 3.3)

project(robocar)

set(CMAKE_CXX_STANDARD 11)

find_package(Threads REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        nav_msgs
        std_msgs
        )
#find_package(OpenCV REQUIRED)
find_library(wiringPi_LIB wiringPi)


#include_directories( ${OpenCV_INCLUDE_DIRS})
include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS})

#file(GLOB SOURCES "robocar_main.cpp" "src/car/Car.cpp"  "src/car/bus/*.cpp" "src/car/sensors/*.cpp"
#        "src/server/Server.cpp" "src/server/communication/*.cpp"
#        "src/slam/*.cpp")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")


add_executable(robocar ${SOURCES})

target_link_libraries(robocar ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(robocar ${wiringPi_LIB} rt -lcrypt)
target_link_libraries(robocar ${catkin_LIBRARIES})

#target_link_libraries( robocar ${OpenCV_LIBS})
