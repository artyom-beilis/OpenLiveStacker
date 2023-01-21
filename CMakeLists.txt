cmake_minimum_required(VERSION 2.8)
project(OpenLiveStacker)

include_directories(include)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2 -g -mavx -msse4.2")
option(OPENCV_ROOT "Path to OpenCV Library" "")

if(OPENCV_ROOT)
    find_library(OPENCV_CORE        opencv_core      HINTS ${OPENCV_ROOT}/lib/ NO_DEFAULT_PATH)
    find_library(OPENCV_IMGPROC     opencv_imgproc   HINTS ${OPENCV_ROOT}/lib/ NO_DEFAULT_PATH)
    find_library(OPENCV_IMGCODECS   opencv_imgcodecs HINTS ${OPENCV_ROOT}/lib/ NO_DEFAULT_PATH)
    find_path(OPENCV_INC opencv2/core.hpp HINTS ${OPENCV_ROOT}/include/opencv4;${OPENCV_ROOT}/include NO_DEFAULT_PATH )
else()
    find_library(OPENCV_CORE        opencv_core      )
    find_library(OPENCV_IMGPROC     opencv_imgproc   )
    find_library(OPENCV_IMGCODECS   opencv_imgcodecs )
    find_path(OPENCV_INC opencv2/core.hpp)
endif()

include_directories(${OPENCV_INC})

add_library(ols SHARED 
    src/uvc_camera.cpp 
    src/camera.cpp 
    src/ols.cpp
    src/video_generator.cpp
    src/tiffmat.cpp
    src/processors.cpp
    src/util.cpp
    src/cv_util.cpp
    )
target_link_libraries(ols tiff uvc cppcms booster ${OPENCV_CORE} ${OPENCV_IMGPROC} ${OPENCV_IMGCODECS})
add_executable(test_camera test/test_camera.cpp)
add_executable(ols_cmd test/ols_cmd.cpp)
add_executable(offline_ols test/offline_sim.cpp)
target_link_libraries(test_camera ols)
target_link_libraries(ols_cmd ols)
target_link_libraries(offline_ols ols)
add_custom_command(TARGET ols POST_BUILD
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/make_db.py ${CMAKE_CURRENT_SOURCE_DIR}/www-data/media/js/jsdb.js
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Building DB"
)
