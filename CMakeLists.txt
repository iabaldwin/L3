cmake_minimum_required(VERSION 2.6.4)

project(L3)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(CMAKE_C_COMPILER "/usr/bin/llvm-gcc" )
    set(CMAKE_CXX_COMPILER "/usr/bin/llvm-g++-4.2" )
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules/")

find_package( Boost COMPONENTS system filesystem unit_test_framework thread REQUIRED )     # Filesystem, smart pointers, ...
include_directories(${Boost_INCLUDE_DIRS})

find_package( HDF5 )                                             # Matlab interoperability  
find_package( GLUT REQUIRED )                                    # Rendering
find_package( GLEW REQUIRED )                                    # Rendering
find_package( OpenGL REQUIRED )                                  # Rendering

#Search for OpenMP, as it doesn't come by default
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    find_package( OpenMP REQUIRED )
    set( CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} )
endif()

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )                    # Matrices

find_package( GSL REQUIRED )                                    # Math,statistics
include_directories( ${GSL_INCLUDE_DIRS} )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/core )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/renderer )

# GLV custom build directory
include_directories( $ENV{GLV_DIR} )
link_directories( $ENV{HOME}/code/build/lib )

find_package( POCO COMPONENTS XML Net Data )
include_directories(${Poco_INCLUDE_DIRS})
link_directories(${Poco_LIBRARY_DIR})

# Core
add_subdirectory( core )

#Visuals
add_subdirectory( renderer )

# Utilities
add_subdirectory( utils )
