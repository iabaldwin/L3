find_path(
    Common_INCLUDE_DIRS 
    Common/estimation/Pose.h
    HINTS
    $ENV{HOME}/code/build/include
    )

find_library(
    Common_LIBRARIES
    NAMES 
    Common
    HINTS
    $ENV{HOME}/code/build/
    PATH_SUFFIXES lib
)


