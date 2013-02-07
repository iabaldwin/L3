cmake_minimum_required(VERSION 2.6.4)

project(L3)

find_package( BOOST )
find_package( HDF5 )
find_package( GLUT )
find_package( OpenGL )

if( Boost_FOUND)
    include_directories( ${BOOST_INCLUDE_DIRS} )
endif()

message( ${Boost_INCLUDE_DIRS} )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/core )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/renderer )

# Core
add_subdirectory( core )

# Graphics
add_subdirectory( renderer )
