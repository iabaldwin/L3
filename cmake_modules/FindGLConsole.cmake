find_path(
    GLCONSOLE_INCLUDE_DIRS 
    GLConsole/GLConsole.h
    HINTS
    $ENV{HOME}/code/build/include
    )

find_library(
    CVARS
    NAMES 
    cvars
    HINTS
    $ENV{HOME}/code/build/
    PATH_SUFFIXES lib
)

#message( ${GLCONSOLE_INCLUDE_DIRS} )
#message( ${CVARS} )

FIND_PACKAGE_HANDLE_STANDARD_ARGS(GLConsole
    REQUIRED_VARS CVARS GLCONSOLE_INCLUDE_DIRS )
