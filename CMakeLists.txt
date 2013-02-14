cmake_minimum_required(VERSION 2.6.4)

project(L3)

find_package( BOOST )       # Filesystem, smart pointers, ...
find_package( HDF5 )        # Matlab interoperability  
find_package( GLUT )        # Rendering
find_package( OpenGL )      # Rendering

if( Boost_FOUND)
    include_directories( ${BOOST_INCLUDE_DIRS} )
endif()

find_package( OpenMP )
set( CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} )

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules/")
find_package( Eigen3 )
include_directories( ${EIGEN3_INCLUDE_DIR} )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/core )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/renderer )

# Core
add_subdirectory( core )

# Graphics
add_subdirectory( renderer )
