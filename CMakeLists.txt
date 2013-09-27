project(L3)

cmake_minimum_required(VERSION 2.6.4)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(CMAKE_C_COMPILER "/usr/bin/llvm-gcc" )
    set(CMAKE_CXX_COMPILER "/usr/bin/llvm-g++-4.2" )
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules/")

# Boost
find_package( Boost COMPONENTS system filesystem unit_test_framework thread chrono regex REQUIRED )     # Filesystem, smart pointers, ...
include_directories(${Boost_INCLUDE_DIRS})

#Search for OpenMP, as it doesn't come by default
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    find_package( OpenMP REQUIRED )
    set( CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} )
endif()

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )                    # Matrices

find_package( GSL REQUIRED )                                    # Math,statistics
include_directories( ${GSL_INCLUDE_DIRS} )

# PCL
include_directories( $ENV{PCL_INCLUDE_DIR} )
link_directories( $ENV{PCL_LIB_DIR} )

# Thread building blocks
if( NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/FindTBB.cmake )
    exec_program( wget ARGS "http://findtbb.googlecode.com/svn/trunk/CMake/FindTBB/FindTBB.cmake -O  ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/FindTBB.cmake" )
endif()
find_package( TBB REQUIRED )
include_directories( ${TBB_INCLUDE_DIRS} )

# PCL
include_directories( $ENV{Bayes_INCLUDE_DIR} )
link_directories( $ENV{Bayes_LIB_DIRECTORY} )

# Core
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/core )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/renderer )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/utils )

include(${CMAKE_SOURCE_DIR}/cmake_modules/PocoConfig.cmake)
#find_package( POCO COMPONENTS XML Net Data )
#include_directories(${Poco_INCLUDE_DIRS})
#link_directories(${Poco_LIBRARY_DIR})
set( POCO_LIBRARIES PocoFoundation )

# GLV custom build directory
#include_directories( $ENV{GLV_DIR} )
#link_directories( $ENV{HOME}/code/build/lib )

# Core
add_subdirectory( core )

#Visuals
add_subdirectory( renderer )

# Learning
add_subdirectory( learning )

# Utilities
add_subdirectory( utils )

# L3
add_subdirectory( app )
