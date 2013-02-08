add_executable( test_writer test_writer.cpp )
target_link_libraries( test_writer L3 )

add_executable( test_dataset test_dataset.cpp )
target_link_libraries( test_dataset L3 boost_system boost_filesystem )

add_executable( test_locale test_locale.cpp )
target_link_libraries( test_locale  config L3 boost_system boost_filesystem )

add_executable( test_pose_homogeneous test_pose_homogeneous.cpp )
target_link_libraries( test_pose_homogeneous  config L3 boost_system boost_filesystem )

add_executable( test_find_closest test_find_closest.cpp )
target_link_libraries( test_find_closest  config L3 boost_system boost_filesystem )

