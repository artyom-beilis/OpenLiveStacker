cmake_minimum_required(VERSION 2.8)
project(OpenLiveStacker)

include_directories(include)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2 -g")
add_library(ols SHARED src/uvc_camera.cpp src/camera.cpp)
target_link_libraries(ols uvc)
add_executable(test_camera test/test_camera.cpp)
target_link_libraries(test_camera ols)
