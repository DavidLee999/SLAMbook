cmake_minimum_required( VERSION 2.8 )
project( vo1 )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( feature_extraction feature_extraction.cpp )
target_link_libraries(feature_extraction ${OpenCV_LIBS} )

add_executable( pose_estimation_2d2d pose_estimation_2d2d.cpp )
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBS} )

add_executable( triangulation triangulation.cpp )
target_link_libraries( triangulation ${OpenCV_LIBS} )
