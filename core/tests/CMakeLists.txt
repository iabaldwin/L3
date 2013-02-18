#add_executable( test_writer test_writer.cpp )
#target_link_libraries( test_writer L3 )

add_executable( test_dataset test_dataset.cpp )
target_link_libraries( test_dataset L3 boost_system boost_filesystem )

#add_executable( test_locale test_locale.cpp )
#target_link_libraries( test_locale config L3 boost_system boost_filesystem )

#add_executable( test_pose_homogeneous test_pose_homogeneous.cpp )
#target_link_libraries( test_pose_homogeneous config L3 boost_system boost_filesystem )

add_executable( test_windower test_windower.cpp )
target_link_libraries( test_windower config L3 boost_system boost_filesystem )

add_executable( test_swathe_builder test_swathe_builder.cpp )
target_link_libraries( test_swathe_builder config L3 boost_system boost_filesystem )

add_executable( test_iterator test_iterator.cpp )
target_link_libraries( test_iterator config L3 boost_system boost_filesystem )

#add_executable( test_pose_factory test_pose_factory.cpp )
#target_link_libraries( test_pose_factory config L3 boost_system boost_filesystem )

#add_executable( test_find_closest test_find_closest.cpp )
#target_link_libraries( test_find_closest config L3 boost_system boost_filesystem )

#add_executable( test_projector test_projector.cpp )
#target_link_libraries( test_projector config L3 boost_system boost_filesystem )

#add_executable( test_point_cloud test_point_cloud.cpp )
#target_link_libraries( test_point_cloud config L3 boost_system boost_filesystem )

#add_executable( test_chain_builder test_chain_builder.cpp )
#target_link_libraries( test_chain_builder config L3 boost_system boost_filesystem )

