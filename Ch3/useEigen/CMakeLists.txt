cmake_minimum_required( VERSION 2.8 )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-O3" )

include_directories( "/usr/local/Cellar/eigen/3.3.4/include/eigen3" )

add_executable( eigenMatirx eigenMatrix.cpp )
