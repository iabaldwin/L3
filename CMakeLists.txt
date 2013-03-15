cmake_minimum_required(VERSION 2.6.4)

project(L3)

find_package( Boost COMPONENTS system filesystem unit_test_framework REQUIRED )     # Filesystem, smart pointers, ...
find_package( HDF5 )                                                                # Matlab interoperability  
find_package( GLUT )                                                                # Rendering
find_package( OpenGL )                                                              # Rendering

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
include_directories( "/Users/ian/code/build/include/pcl-1.6/" )
include_directories( "/usr/local/include/vtk-5.10/" )

link_directories( "/Users/ian/code/build/lib/" )

find_package( POCO COMPONENTS XML Net Data )
include_directories(${Poco_INCLUDE_DIRS})
link_directories(${Poco_LIBRARY_DIR})

# Core
add_subdirectory( core )

#Visuals
add_subdirectory( renderer )

# Utilities
#add_subdirectory( utils )
