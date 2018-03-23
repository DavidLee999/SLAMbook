cmake_minimum_required( VERSION 2.8 )

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11" )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

include_directories( "/usr/local/Cellar/eigen/3.3.4/include/eigen3" )

find_package( PCL REQUIRED COMPONENT common io filters )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

find_package( octomap REQUIRED )
include_directories( ${OCTOMAP_INCLUDE_DIRS} )

add_executable( pointcloud_mapping pointcloud_mapping.cpp )
target_link_libraries( pointcloud_mapping ${OpenCV_LIBS} ${PCL_LIBRARIES} )

#/usr/local/Cellar/pcl/1.8.1/pcl_viewer.app/Contents/MacOS/pcl_viewer