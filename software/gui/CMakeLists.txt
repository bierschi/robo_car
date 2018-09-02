cmake_minimum_required(VERSION 3.7)
project(robocar_gui)

set(CMAKE_CXX_STANDARD 11)

# create executable in /build directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")

# instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)

# create code form a list of Qt designer ui files
set(CMAKE_AUTOUIC ON)

# find the Qt Widgets Core libraries
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

# populate a CMake variable with the sources
set(main_gui_src
        src/mainwindow.ui
        src/mainwindow.cpp
        src/mainwindow.h
        src/client.h
        src/client.cpp
        main_gui.cpp)

# tell CMake to create a main_gui executable
add_executable(main_gui ${main_gui_src})


target_link_libraries(main_gui Qt5::Widgets)