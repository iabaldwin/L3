add_executable( test_include_order test_include_order.cpp )
target_link_libraries( test_include_order L3 ${Boost_LIBRARIES}   )

add_executable( test_dataset test_dataset.cpp )
target_link_libraries( test_dataset L3 ${Boost_LIBRARIES}   )

add_executable( test_utils test_utils.cpp )
target_link_libraries( test_utils L3 ${Boost_LIBRARIES}  )

add_executable( test_locale test_locale.cpp )
target_link_libraries( test_locale config L3 ${Boost_LIBRARIES}  )

add_executable( test_pose_homogeneous test_pose_homogeneous.cpp )
target_link_libraries( test_pose_homogeneous config L3 ${Boost_LIBRARIES}  )

add_executable( test_windower test_windower.cpp )
target_link_libraries( test_windower config L3 ${Boost_LIBRARIES}  )

add_executable( test_swathe_builder test_swathe_builder.cpp )
target_link_libraries( test_swathe_builder config L3 ${Boost_LIBRARIES}  )

add_executable( test_iterator test_iterator.cpp )
target_link_libraries( test_iterator config L3 ${Boost_LIBRARIES}  )

add_executable( test_reader_bandwidth test_reader_bandwidth.cpp )
target_link_libraries( test_reader_bandwidth config L3 ${Boost_LIBRARIES}  )

add_executable( test_binary_reader test_binary_reader.cpp )
target_link_libraries( test_binary_reader config L3 ${Boost_LIBRARIES}  )

add_executable( test_pose_factory test_pose_factory.cpp )
target_link_libraries( test_pose_factory config L3 ${Boost_LIBRARIES}  )

add_executable( test_windower_factory test_windower_factory.cpp )
target_link_libraries( test_windower_factory config L3 ${Boost_LIBRARIES}  )

add_executable( test_projector test_projector.cpp )
target_link_libraries( test_projector config L3 ${Boost_LIBRARIES}  )

add_executable( test_point_cloud test_point_cloud.cpp )
target_link_libraries( test_point_cloud config L3 ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY}  ${Boost_FILESYSTEM_LIBRARY})

add_executable( test_point_cloud_join test_point_cloud_join.cpp )
target_link_libraries( test_point_cloud_join config L3 ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY}  ${Boost_FILESYSTEM_LIBRARY} )

add_executable( test_chain_builder test_chain_builder.cpp )
target_link_libraries( test_chain_builder config L3 ${Boost_LIBRARIES}  )

add_executable( test_library_adapters test_library_adapters.cpp )
target_link_libraries( test_library_adapters config L3 ${Boost_LIBRARIES}  )

add_executable( test_pipeline test_pipeline.cpp )
target_link_libraries( test_pipeline config L3 ${Boost_LIBRARIES}  )

add_executable( test_readers test_readers.cpp )
target_link_libraries( test_readers config L3 ${Boost_LIBRARIES}  )

add_executable( test_sequential_file_reader test_sequential_file_reader.cpp )
target_link_libraries( test_sequential_file_reader config L3 ${Boost_LIBRARIES}  )

add_executable( test_length_estimator test_length_estimator.cpp )
target_link_libraries( test_length_estimator config L3 ${Boost_LIBRARIES}  )

add_executable( test_experience_builder test_experience_builder.cpp )
target_link_libraries( test_experience_builder config L3 ${Boost_LIBRARIES}  )

add_executable( test_experience_loader test_experience_loader.cpp )
target_link_libraries( test_experience_loader config L3 ${Boost_LIBRARIES}  )

add_executable( test_experience_pipeline test_experience_pipeline.cpp )
target_link_libraries( test_experience_pipeline config L3 ${Boost_LIBRARIES}  )

add_executable( test_experience_query test_experience_query.cpp )
target_link_libraries( test_experience_query config L3 ${Boost_LIBRARIES}  )


