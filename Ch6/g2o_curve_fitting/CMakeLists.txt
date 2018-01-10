cmake_minimum_required( VERSION 2.8 )
project( g2o_curve_fitting )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

set( LINK_DIR /usr/local/lib )
link_directories( ${LINK_DIR} )

find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )

include_directories( "/usr/local/Cellar/eigen/3.3.4/include/eigen3" )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( g2o_curve_fitting main.cpp )

target_link_libraries( g2o_curve_fitting ${OpenCV_LIBS} g2o_core g2o_stuff )
