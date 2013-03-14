add_library( L3 Dataset.cpp Datatypes.cpp)
target_link_libraries( L3 config++ gsl PocoFoundation glog )

add_subdirectory( tests )
add_subdirectory( apps )
