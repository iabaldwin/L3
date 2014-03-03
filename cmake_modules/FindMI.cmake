# - Try to find libMI
#   Once done this will define
#       MI_FOUND        - System has libMI
#       MI_INCLUDE_DIRS - Include directories
#       MI_LIBRARY_DIRS - The libraries locations
#       MI_LIBRARIES    - The libraries needed 

find_path(MI_INCLUDE_DIRS BayesFilter/bayesFlt.hpp
    HINTS 
        "/usr/local/include" 
        "/usr/include" 
        "$ENV{HOME}/code/build/include"
        "$ENV{LIBMI_INCLUDE_DIRS}" )


find_library(MI_LIBRARIES 
    NAMES 
        libMI 
        MI
    HINTS 
        "/usr/local/lib" 
        "/usr/lib"
        "$ENV{HOME}/code/build/lib" )

find_path(MI_LIBRARY_DIRS  libMI.a
    HINTS 
        "/usr/local/lib" 
        "/usr/lib"
        "$ENV{HOME}/code/build/lib" )


