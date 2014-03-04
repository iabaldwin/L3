# - Try to find Bayes++
#   Once done this will define
#       BAYES++_FOUND - System has Bayes++
#       BAYES++_INCLUDE_DIRS - The Bayes++ include directories
#       BAYES++_LIBRARIES - The libraries needed to use Bayes++
#       BAYES++_DEFINITIONS - Compiler switches required for using Bayes++

find_path(BAYES++_INCLUDE_DIRS BayesFilter/bayesFlt.hpp
    HINTS 
        "/usr/local/include" 
        "/usr/include" 
        "$ENV{HOME}/code/build/include"
        "$ENV{LIBBAYES++_INCLUDE_DIRS}" )

find_library(BAYES++_LIBRARIES 
    NAMES 
        libBayesFilter 
        BayesFilter
    HINTS 
        "/usr/local/lib" 
        "/usr/lib"
        "$ENV{HOME}/code/build/lib" )
