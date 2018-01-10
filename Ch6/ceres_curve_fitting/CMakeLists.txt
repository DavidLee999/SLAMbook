cmake_minimum_required( VERSION 2.8 )
project( ceres_curve_fitting )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# include_directories( "/usr/local/Cellar/eigen/3.3.4/include/eigen3" )

find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( curve_fittig main.cpp )

target_link_libraries( curve_fittig ${CERES_LIBRARIES} ${OpenCV_LIBRARIES} )
