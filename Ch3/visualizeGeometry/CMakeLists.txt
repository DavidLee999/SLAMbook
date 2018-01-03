cmake_minimum_required( VERSION 2.8 )
project( visualizeGeometry )

set( CMAKE_CXX_FLAGS "-std=c++11" )

include_directories( "/usr/local/Cellar/eigen/3.3.4/include/eigen3" )

find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

add_executable( visualizeGeometry visualizeGeometry.cpp )
target_link_libraries(visualizeGeometry ${Pangolin_LIBRARIES} )
