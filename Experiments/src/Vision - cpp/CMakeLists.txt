cmake_minimum_required(VERSION 2.6)
project(Vision_Study)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(freenect2 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${freenect2_INCLUDE_DIR})
include_directories(${PROJECT_BINARY_DIR})

add_executable(kinectTest kinectTest.cpp)
target_link_libraries(kinectTest ${OpenCV_LIBS} ${freenect2_LIBRARY})

add_executable(camTest camTest.c)
target_link_libraries(camTest freenect)