project(L3)

cmake_minimum_required(VERSION 2.6.4)

include( CTest )

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(CMAKE_C_COMPILER "/usr/bin/llvm-gcc" )
    set(CMAKE_CXX_COMPILER "/usr/bin/llvm-g++-4.2" )
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules/")
set( PCL_DIR "$ENV{HOME}/code/thirdparty/PCL.github.build/" )

# Search for OpenMP, as it doesn't come by default
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    find_package( OpenMP REQUIRED )
    set( CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} )
endif()

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )                    # Matrices

find_package( GSL REQUIRED )                                    # Math,statistics
include_directories( ${GSL_INCLUDE_DIRS} )

if( NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/FindTBB.cmake )    # Thread building blocks
    exec_program( wget ARGS "http://findtbb.googlecode.com/svn/trunk/CMake/FindTBB/FindTBB.cmake -O  ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/FindTBB.cmake" )
endif()
find_package( TBB REQUIRED )
include_directories( ${TBB_INCLUDE_DIRS} )

find_package( BAYES++ REQUIRED )                                # Bayesian inference
include_directories( ${BAYES++_INCLUDE_DIRS} )

find_package( MI REQUIRED )                                     # Information
include_directories(${MI_INCLUDE_DIRS})
link_directories(${MI_LIBRARY_DIRS})

find_package(PCL REQUIRED COMPONENTS common io registration )   # Various point-cloud tools
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

set( Boost_USE_STATIC_LIBS ON )
find_package( Boost COMPONENTS system filesystem thread regex chrono python REQUIRED )     # Filesystem, smart pointers, ...
include_directories(${Boost_INCLUDE_DIRS})

# POCO
include(${CMAKE_SOURCE_DIR}/cmake_modules/PocoConfig.cmake)     # Sockets, net, ...
include_directories( ${Poco_INCLUDE_DIRS} )
link_directories( ${Poco_LIBRARY_DIRS} )
set( poco_libraries PocoFoundation PocoNet )

# GLV custom build directory
#include_directories( $ENV{GLV_DIR} )
#link_directories( $ENV{HOME}/code/build/lib )

find_package( Common REQUIRED )
include_directories( ${COMMON_INCLUDE_DIRS} )

# Core
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/core )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/renderer )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/utils )

# Core
#add_subdirectory( core )

## Visuals
#add_subdirectory( renderer )

# Interfaces
add_subdirectory( interfaces )

# Learning
#add_subdirectory( learning )

## Utilities
#add_subdirectory( utils )

## L3
#add_subdirectory( app )
