cmake_minimum_required(VERSION 2.8)
project(OpenLiveStacker)

include_directories(include)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2 -g")

find_library(OPENCV_CORE        opencv_core      HINTS /opt/opencv-4.6/lib/ NO_DEFAULT_PATH)
find_library(OPENCV_IMGPROC     opencv_imgproc   HINTS /opt/opencv-4.6/lib/ NO_DEFAULT_PATH)
find_library(OPENCV_IMGCODECS   opencv_imgcodecs HINTS /opt/opencv-4.6/lib/ NO_DEFAULT_PATH)
find_path(OPENCV_INC opencv2/core.hpp HINTS /opt/opencv-4.6/include/opencv4 NO_DEFAULT_PATH )

include_directories(${OPENCV_INC})

add_library(ols SHARED src/uvc_camera.cpp src/camera.cpp src/ols.cpp)
target_link_libraries(ols uvc cppcms booster ${OPENCV_CORE} ${OPENCV_IMGPROC} ${OPENCV_IMGCODECS})
add_executable(test_camera test/test_camera.cpp)
add_executable(ols_cmd test/ols_cmd.cpp)
target_link_libraries(test_camera ols)
target_link_libraries(ols_cmd ols)
