cmake_minimum_required( VERSION 2.8 )

project( HelloSLAM )

add_executable( helloSLAM helloSLAM.cpp )
add_executable( useHello useHello.cpp )


add_library( hello libHelloSLAM.cpp )
add_library( hello_shared libHelloSLAM.cpp )

target_link_libraries(useHello hello_shared )

set( CMAKE_BUILD_TYPE "Debug" )
