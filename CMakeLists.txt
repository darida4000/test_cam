cmake_minimum_required(VERSION 2.8)
project( test_cam )
find_package( OpenCV REQUIRED )
add_executable( test_cam test_cam.cpp  )

target_link_libraries( test_cam   opencv_core opencv_imgproc opencv_video opencv_calib3d ${EXTERNAL_LIBS})
include_directories(/usr/incude)
