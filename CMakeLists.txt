SET(CMAKE_CXX_COMPILER g++)

project(L3)

cmake_minimum_required(VERSION 2.6.4)
set (CMAKE_CXX_STANDARD 11)

include(CTest)

# Search for OpenMP, as it doesn't come by default
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    find_package(OpenMP REQUIRED)
    set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS})
endif()

add_definitions(-DCOMPILE_C)
add_definitions(-Wno-deprecated-declarations)

link_directories($ENV{CMAKE_LINK_DIRECTORIES})
include_directories($ENV{CMAKE_INCLUDE_DIRECTORIES})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "$ENV{CMAKE_CACHE}/find")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})                    # Matrices

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

find_package(GSL REQUIRED)                                    # Math,statistics
include_directories(${GSL_INCLUDE_DIRS})

find_package(TBB REQUIRED)                                    # Concurrency
include_directories(${TBB_INCLUDE_DIRS})

find_package(BAYES++ REQUIRED)                                # Bayesian inference
include_directories(${BAYES++_INCLUDE_DIRS})

find_library(CONFIG NAME libconfig++.a HINTS $ENV{CMAKE_LINK_DIRECTORIES})  # Config files
find_library(GLV_LIBRARY NAME libGLV.a HINTS $ENV{CMAKE_LINK_DIRECTORIES})  # Rendering

find_package(MI REQUIRED)                                     # Information
include_directories(${MI_INCLUDE_DIRS})
link_directories(${MI_LIBRARY_DIRS})

find_package(PCL REQUIRED COMPONENTS common io registration)   # Various point-cloud tools
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost COMPONENTS system filesystem thread regex chrono python REQUIRED)     # Filesystem, smart pointers, ...
include_directories(${Boost_INCLUDE_DIRS})

find_package(Poco COMPONENTS Net Data Util Foundation)                        # Timing tools, ..
link_directories(${Poco_LIBRARY_DIRS})
include_directories(${Poco_INCLUDE_DIRS})

# Core
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/core)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/renderer)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/utils)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Common)

# Core
add_subdirectory(common)

# Core
add_subdirectory(core)

# Visuals
add_subdirectory(renderer)

# Interfaces
add_subdirectory(interfaces)

# Learning
add_subdirectory(learning)

# Utilities
add_subdirectory(utils)

## L3
add_subdirectory(app)
