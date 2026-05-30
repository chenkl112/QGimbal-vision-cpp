cmake_minimum_required(VERSION 3.16)

project(QGimbalVisionCpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio)

add_executable(qgimbal_vision
    src/main.cpp
    src/rectangle_detector.cpp
    src/pid.cpp
    src/tracker_control.cpp
    src/gimbal_serial.cpp
)

target_include_directories(qgimbal_vision
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(qgimbal_vision PRIVATE ${OpenCV_LIBS})

if(MSVC)
    target_compile_options(qgimbal_vision PRIVATE /W4)
else()
    target_compile_options(qgimbal_vision PRIVATE -Wall -Wextra -Wpedantic)
endif()
