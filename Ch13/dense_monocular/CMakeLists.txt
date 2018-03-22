cmake_minimum_required( VERSION 2.8 )
project( dense_monocular )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

include_directories( "/usr/local/Cellar/eigen/3.3.4/include/eigen3" )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

set( THIRD_PARTY_LIBS ${OpenCV_LIBS} ${Sophus_LIBRARIES} )

add_executable( dense_mapping dense_mapping.cpp )
target_link_libraries( dense_mapping ${THIRD_PARTY_LIBS} )
