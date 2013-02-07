#add_executable( load_poses load_poses.cpp )
#add_executable( load_scans load_scans.cpp)

add_library( L3  Reader.cpp )

add_subdirectory( tests )
