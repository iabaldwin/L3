add_executable( load_poses load_poses.cpp )
target_link_libraries( load_poses L3 boost_system boost_filesystem )

add_executable( load_scans load_scans.cpp )
target_link_libraries( load_scans L3 boost_system boost_filesystem )

add_executable( experience_generator experience_generator.cpp )
target_link_libraries( experience_generator config L3 boost_system boost_filesystem )

add_executable( test_config test_config.cpp )
target_link_libraries( test_config config L3 boost_system boost_filesystem )

