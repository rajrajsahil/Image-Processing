cmake_minimum_required(VERSION 3.5.1)
project(kohli)
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(raj test.cpp)
target_link_libraries(raj ${OpenCV_LIBS})
