# - Custom library locations

find_path(COMMON_INCLUDE_DIR Common/imagery/Imagery.h
          HINTS $ENV{HOME}/code/build/include )

find_library(COMMON_LIBRARY 
            NAMES libCommon Common
            HINTS $ENV{HOME}/code/build/lib )

set(COMMON_LIBRARIES ${COMMON_LIBRARY} )
set(COMMON_INCLUDE_DIRS ${COMMON_INCLUDE_DIR} )

mark_as_advanced(COMMON_INCLUDE_DIR COMMON_LIBRARY )
