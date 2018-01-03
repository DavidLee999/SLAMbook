cmake_minimum_required( VERSION 2.8 )
project( geometry )

include_directories( "/usr/local/Cellar/eigen/3.3.4/include/eigen3" )

add_executable( eigenGeometry eigenGeometry.cpp )
