cmake_minimum_required( VERSION 2.8 )
project( vo1 )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

set( LINK_DIR /usr/local/lib )
link_directories( ${LINK_DIR} )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )

find_package( CSparse REQUIRED )
include_directories( ${CSPARSE_INCLUDE_DIR} )

include_directories( "/usr/local/Cellar/eigen/3.3.4/include/eigen3" )

add_executable( feature_extraction feature_extraction.cpp )
target_link_libraries(feature_extraction ${OpenCV_LIBS} )

add_executable( pose_estimation_2d2d pose_estimation_2d2d.cpp )
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBS} )

add_executable( triangulation triangulation.cpp )
target_link_libraries( triangulation ${OpenCV_LIBS} )

add_executable(pose_estimation_3d2d pose_estimation_3d2d.cpp )
target_link_libraries(pose_estimation_3d2d ${OpenCV_LIBS} ${CSPARSE_LIBRARY} g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension )
