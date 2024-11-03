cmake_minimum_required(VERSION 2.8)
project(OpenLiveStacker)

include_directories(include)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2 -g")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -rdynamic")
option(OPENCV_ROOT "Path to OpenCV Library" "")
option(NO_SSE4 "Disable SSE 4.1/4.3 on x86/x86_64 builds" OFF)
option(USE_INDI_AS_LIBRARY "Use special variand of indi - embedded server/client/deriver" OFF)

find_library(ASI_CAMERA_LIB ASICamera2)
find_path(ASI_CAMERA_HDR ASICamera2.h)

if(USE_INDI_AS_LIBRARY)
    find_library(INDI_LIB1 indi_unified_client_server HINT /opt/indi_as_library/lib)
    find_library(INDI_LIB2 indi_unified HINT /opt/indi_as_library/lib)
    set(INDI_LIB ${INDI_LIB1} ${INDI_LIB2})
    find_path(INDI_HEADER libindi/indiserver_lib.h 
                HINT ${CMAKE_FIND_ROOT_PATH}/include
                HINT /opt/indi_as_library/include)
    include_directories(${INDI_HEADER}/libindi)
    add_definitions(-DINDI_AS_LIBRARY)
else()
    find_library(INDI_LIB indiclient)
    find_path(INDI_HEADER libindi/baseclient.h)
    include_directories(${INDI_HEADER})
endif()

find_library(INDIGO_LIB indigo)
find_path(INDIGO_HEADER indigo/indigo_bus.h)

find_library(TOUPCAM_CAMERA_LIB toupcam)
find_path(TOUPCAM_CAMERA_HDR toupcam.h)

find_library(LIBTIFF tiff)
find_library(LIBCPPCMS cppcms)
find_library(LIBBOOSTER booster)
find_library(LIBUVC uvc)
find_library(LIBRAW_LIB raw_r)
find_path(LIBRAW_HDR libraw/libraw.h)


find_path(LIBCURL_HDR curl/curl.h)
find_path(LIBZ_HDR zlib.h)
find_path(LIBCPPCMS_HRD cppcms/application.h)
find_path(LIBCAMERA2_HDR camera/NdkCameraDevice.h)
find_path(LIBMEDIANDK_HDR media/NdkImageReader.h)
find_library(LIBCURL curl)
find_library(LIBZ z)
find_library(LIBCAMERA2_LIB camera2ndk)
find_library(LIBMEDIANDK_LIB mediandk)

find_path(LIBGPHOTO2_HDR gphoto2/gphoto2.h)
find_library(LIBGPHOTO2_LIB gphoto2)
find_library(LIBGPHOTO2_PORT_LIB gphoto2_port)



set(WITH_SSE OFF)
set(OLS_EXTRA )
set(OLS_LIB_EXTRA )


include_directories(${LIBCPPCMS_HRD})

if(LIBCURL_HDR AND LIBCURL)
    set(WITH_CURL ON)
    add_definitions(-DWITH_CURL)
    message("-- ASTAP DB Download enabled")
else()
    message("-- ASTAP DB Download DISABLED")
    set(WITH_CURL OFF)
endif()



if(${CMAKE_SYSTEM_NAME} STREQUAL "Android")
    set(OLS_EXTRA src/android_main.cpp)
    set(ANDROID_BUILD ON)
    add_definitions(-DANDROID_SUPPORT)
    add_definitions(-DOLS_ARCH="${ANDROID_ABI}")
    find_library(LIBUSB usb1.0)
    if(${ANDROID_ABI} STREQUAL "x86_64" OR ${ANDROID_ABI} STREQUAL "x86")
        set(WITH_SSE ON)
        if(${ANDROID_ABI} STREQUAL "x86")
            set(NO_SSE4 ON)
        endif()
        add_definitions(-DUSE_CV_SIMD)
    elseif(${ANDROID_ABI} STREQUAL "arm64-v8a") # No simd for armeabi-v7a
        add_definitions(-DUSE_CV_SIMD)
        add_definitions(-DUSE_ARM_SIMD)
    endif()
else()
    message("CPU: ${CMAKE_SYSTEM_PROCESSOR}")
    add_definitions(-DOLS_ARCH="${CMAKE_SYSTEM_PROCESSOR}")
    if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64" OR ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86") 
        add_definitions(-DUSE_CV_SIMD)
        set(WITH_SSE ON)
    endif()        
    set(ANDROID_BUILD OFF)
endif()


if(NOT ANDROID_BUILD)
    find_path(LIBCFITSIO_HDR fitsio.h)
    find_library(LIBCFITSIO cfitsio)
endif()    


if(WITH_SSE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2")
    if(NOT NO_SSE4)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.1 -msse4.2")
    endif()
endif()        



if(OPENCV_ROOT)
    message("Searching lib in ${OPENCV_ROOT}")
    find_library(OPENCV_CORE        opencv_core      PATHS ${OPENCV_ROOT}/lib/ NO_DEFAULT_PATH)
    find_library(OPENCV_IMGPROC     opencv_imgproc   PATHS ${OPENCV_ROOT}/lib/ NO_DEFAULT_PATH)
    find_library(OPENCV_IMGCODECS   opencv_imgcodecs PATHS ${OPENCV_ROOT}/lib/ NO_DEFAULT_PATH)
    find_path(OPENCV_INC opencv2/core.hpp PATHS ${OPENCV_ROOT}/include/opencv4;${OPENCV_ROOT}/include NO_DEFAULT_PATH )
else()
    find_library(OPENCV_CORE        opencv_core      )
    find_library(OPENCV_IMGPROC     opencv_imgproc   )
    find_library(OPENCV_IMGCODECS   opencv_imgcodecs )
    find_path(OPENCV_INC_A opencv2/core.hpp)
    if(NOT OPENCV_INC_A)
        find_path(OPENCV_INC_B opencv4/opencv2/core.hpp)
        if(OPENCV_INC_B)
            set(OPENCV_INC "${OPENCV_INC_B}/opencv4")
        else()
            message("Can't find OpenCV include")
        endif()
    else()
        set(OPENCV_INC "${OPENCV_INC_A}")
    endif()

endif()

message("- OpenCV Include Path ${OPENCV_INC}")
include_directories(${OPENCV_INC})
if(WITH_CURL)
    include_directories(${LIBCURL_HDR})
endif()
include_directories(${LIBZ_HDR})


if(ASI_CAMERA_HDR AND ASI_CAMERA_LIB)
    include_directories(${ASI_CAMERA_HDR})
    add_library(ols_driver_asi SHARED src/asi_camera.cpp)
    target_link_libraries(ols_driver_asi ${ASI_CAMERA_LIB})
    install(TARGETS ols_driver_asi
            LIBRARY DESTINATION lib
    )
    message("- Building ASI Driver")
else()
    message("- No ASI SDK header/lib found")
endif()

if(TOUPCAM_CAMERA_HDR AND TOUPCAM_CAMERA_LIB)
    include_directories(${TOUPCAM_CAMERA_HDR})
    add_library(ols_driver_toup SHARED src/toup_camera.cpp)
    target_link_libraries(ols_driver_toup ${TOUPCAM_CAMERA_LIB})
    install(TARGETS ols_driver_toup
            LIBRARY DESTINATION lib
    )
    message("- Building Toupcam Driver")
else()
    message("- No Toupcam SDK header/lib found")
endif()

if(LIBCFITSIO AND LIBCFITSIO_HDR)
    set(OLS_EXTRA ${OLS_EXTRA} src/fitsmat.cpp)
    set(OLS_LIB_EXTRA ${LIBCFITSIO})
endif()

if(LIBCFITSIO AND LIBCFITSIO_HDR)
    include_directories(${LIBCFITSIO_HDR})
    add_definitions(-DWITH_CFITSIO)
    message("- Building OLS with fits support")
else()
    message("- Building OLS without fits support")
endif()


if(INDI_LIB AND INDI_HEADER)
    message(" - Building OLS with INDI Mount Support")
    set(OLS_EXTRA ${OLS_EXTRA} src/indi_mount.cpp)
    add_definitions(-DWITH_INDI)
else()
    message(" - Building OLS without INDI Mount Support")
endif()




add_library(ols SHARED 
    src/camera.cpp 
    src/ols.cpp
    src/mount_ctl.cpp
    src/mount.cpp
    src/video_generator.cpp
    src/tiffmat.cpp
    src/debug_save_processor.cpp
    src/post_processor.cpp
    src/pre_processor.cpp
    src/stacker_processor.cpp
    src/common_utils.cpp
    src/util.cpp
    src/plate_solver.cpp
    src/server_sent_events.cpp
    src/downloader.cpp
    src/allocator.cpp
    ${OLS_EXTRA}
    )

target_link_libraries(ols dl ${LIBTIFF} ${LIBCPPCMS} ${LIBZ} ${LIBBOOSTER} ${OPENCV_CORE} ${OPENCV_IMGPROC} ${OPENCV_IMGCODECS} ${OLS_LIB_EXTRA} ${INDI_LIB})


if(ANDROID_BUILD)
    target_link_libraries(ols log)
endif()

if(WITH_CURL)
    target_link_libraries(ols ${LIBCURL} ${LIBZ})
endif()


if(LIBUVC)
    add_library(ols_driver_uvc SHARED src/uvc_camera.cpp)
    target_link_libraries(ols_driver_uvc ols ${LIBUVC})    
    if(ANDROID_BUILD)   
        target_link_libraries(ols_driver_uvc ${LIBUSB})
    endif()
    message("- Building UVC Driver")
    install(TARGETS ols_driver_uvc
            LIBRARY DESTINATION lib
    )
else()
    message("- No libuvc, skipping UVC Driver")
endif()

add_library(ols_driver_sim SHARED src/sim_camera.cpp)
target_link_libraries(ols_driver_sim ols ${OPENCV_IMGCODECS})




if(NOT ANDROID_BUILD AND NOT APPLE)  
    add_library(ols_driver_wdir SHARED src/wdir_camera.cpp)
    target_link_libraries(ols_driver_wdir ols ${OPENCV_CORE} ${OPENCV_IMGPROC} ${OPENCV_IMGCODECS} ${LIBBOOSTER})
    if(LIBRAW_LIB AND LIBRAW_HDR)
        include_directories(${LIBRAW_HDR})
        target_compile_definitions(ols_driver_wdir PRIVATE WITH_LIBRAW)
        target_link_libraries(ols_driver_wdir ${LIBRAW_LIB})
        message("- Building WDir driver with libraw")
        install(TARGETS ols_driver_wdir
                LIBRARY DESTINATION lib
        )
    else()
        message("- Building WDir driver without libraw")
    endif()
endif()


if(INDIGO_HEADER AND INDIGO_LIB AND LIBCFITSIO AND LIBCFITSIO_HDR)
    include_directories(${INDIGO_HEADER})
    add_library(ols_driver_indigo SHARED src/indigo_camera.cpp)
    target_link_libraries(ols_driver_indigo ${INDIGO_LIB})
    message(" -- Building indigo camera driver")
else()
    message(" -- Indigo camera driver disabled")
endif()

if(INDI_HEADER AND INDI_LIB AND LIBCFITSIO AND LIBCFITSIO_HDR AND NOT ANDROID_BUILD)
    include_directories(${INDI_HEADER})
    add_library(ols_driver_indi SHARED src/indi_camera.cpp)
    target_link_libraries(ols_driver_indi ${INDI_LIB})
    message(" -- Building indi camera driver")
else()
    message(" -- Indi camera driver disabled")
endif()




if(LIBGPHOTO2_HDR AND LIBGPHOTO2_LIB AND LIBGPHOTO2_PORT_LIB)
    include_directories(${LIBGPHOTO2_HDR})
    add_library(ols_driver_gphoto2 SHARED src/gphoto2_camera.cpp)
    target_link_libraries(ols_driver_gphoto2 ols ${LIBGPHOTO2_LIB} ${LIBGPHOTO2_PORT_LIB} ${OPENCV_CORE} ${OPENCV_IMGCODECS} ${OPENCV_IMGPROC})

    if(LIBRAW_LIB AND LIBRAW_HDR)
        include_directories(${LIBRAW_HDR})
        set_target_properties(ols_driver_gphoto2 PROPERTIES COMPILE_DEFINITIONS WITH_LIBRAW)
        target_link_libraries(ols_driver_gphoto2 ${LIBRAW_LIB})
        message("- Building GPhoto2 Driver with libraw")
    else()
        message("- Building GPhoto2 Driver without libraw")
    endif()


    install(TARGETS ols_driver_gphoto2
            LIBRARY DESTINATION lib
    )
else()
    message("- No GPhoto2 header/lib found")
endif()

if(ANDROID_BUILD)
    if(LIBCAMERA2_HDR AND LIBCAMERA2_LIB AND LIBMEDIANDK_HDR AND LIBMEDIANDK_LIB)
        include_directories(LIBCAMERA2_HDR)
        include_directories(LIBMEDIANDK_HDR)
        add_library(ols_driver_android SHARED src/android_camera.cpp)
        target_link_libraries(ols_driver_android ols ${OPENCV_CORE} ${LIBCAMERA2_LIB} ${LIBMEDIANDK_LIB} android)
        message("- Building Android camera driver ")
        install(TARGETS ols_driver_android
                LIBRARY DESTINATION lib
        )
    else()
        message("- No Android Ndk Camera2 header/lib found, disabling android camera support")
    endif()
endif()

install(TARGETS ols ols_driver_sim 
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
)

if(NOT ANDROID_BUILD)
    include_directories(src)
    add_executable(test_indigo_format_parsing test/test_indigo_format_parsing.cpp)
    add_executable(test_camera test/test_camera.cpp)
    add_executable(ols_cmd src/ols_cmd.cpp)
    add_executable(offline_ols test/offline_sim.cpp)
    add_executable(ols_plate_solver_test test/plate_solver_test.cpp)
    target_link_libraries(test_camera ols)
    target_link_libraries(ols_cmd ols)
    target_link_libraries(offline_ols ols)
    target_link_libraries(ols_plate_solver_test ols ${OPENCV_CORE} ${OPENCV_IMGPROC} ${OPENCV_IMGCODECS})
    target_link_libraries(test_indigo_format_parsing ols)
    install(TARGETS ols_driver_sim ols_cmd offline_ols
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
    )
endif()



add_custom_command(TARGET ols POST_BUILD
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/make_db.py ${CMAKE_CURRENT_SOURCE_DIR}/www-data/media/js/jsdb.js
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Building DB"
)
