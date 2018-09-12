cmake_minimum_required(VERSION 3.7)
project(robocar)

set(CMAKE_CXX_STANDARD 11)

find_package(Threads REQUIRED)
#find_package(OpenCV REQUIRED)

#include_directories( ${OpenCV_INCLUDE_DIRS})
include_directories(include)

file(GLOB SOURCES "src/robocar_main.cpp" "src/comm/*.cpp" "src/sensors/*cpp")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")


add_executable(robocar ${SOURCES})
target_link_libraries(robocar ${CMAKE_THREAD_LIBS_INIT})

#target_link_libraries( robocar ${OpenCV_LIBS})

#add_executable(main_ServerSocket src/main_serversocket.cpp "src/comm/ServerSocket.cpp" "include/comm/ServerSocket.h" "src/comm/Socket.cpp" "include/comm/Socket.h" "src/sensors/PCA9685.cpp" "include/sensors/PCA9685.h" "src/comm/I2C.cpp" "include/comm/I2C.h")
#add_executable(main_ClientSocket ../client/main_clientsocket.cpp "../client/src/ClientSocket.cpp" "../client/include/ClientSocket.h" "src/comm/Socket.cpp" "include/comm/Socket.h")
#add_executable(main_i2c src/main_i2c.cpp "src/sensors/PCA9685.cpp" "include/sensors/PCA9685.h" "src/comm/I2C.cpp" "include/comm/I2C.h")



#add_executable(main_server src/main_server.cpp "src/comm/server.cpp" "include/comm/server.h")
#add_executable(main_client src/main_client.cpp "src/comm/client.cpp" "include/comm/client.h")
