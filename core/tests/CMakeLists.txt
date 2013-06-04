#add_executable( test_libconfig test_libconfig.cpp )
#target_link_libraries( test_libconfig config L3 ${Boost_LIBRARIES} )

#add_executable( test_timer test_timer.cpp )
#target_link_libraries( test_timer config L3 ${Boost_LIBRARIES} )

#add_executable( test_point_cloud_transform_vs_translate test_point_cloud_transform_vs_translate.cpp )
#target_link_libraries( test_point_cloud_transform_vs_translate config L3 ${Boost_LIBRARIES} )

#add_executable( test_conversions test_conversions.cpp )
#target_link_libraries( test_conversions config L3 ${Boost_LIBRARIES} )

#add_executable( test_kl_computation test_kl_computation.cpp )
#target_link_libraries( test_kl_computation L3 ${Boost_LIBRARIES}   )

#add_executable( test_minimisation test_minimisation.cpp )
#target_link_libraries( test_minimisation L3 ${Boost_LIBRARIES}   )

#add_executable( test_configuration test_configuration.cpp )
#target_link_libraries( test_configuration L3 ${Boost_LIBRARIES}   )

#add_executable( test_dataset test_dataset.cpp )
#target_link_libraries( test_dataset L3 ${Boost_LIBRARIES}   )

#add_executable( test_point_cloud test_point_cloud.cpp )
#target_link_libraries( test_point_cloud config L3 ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY}  ${Boost_FILESYSTEM_LIBRARY})

#add_executable( test_point_cloud_copy test_point_cloud_copy.cpp )
#target_link_libraries( test_point_cloud_copy config L3 ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY}  ${Boost_FILESYSTEM_LIBRARY})

#add_executable( test_point_cloud_transform test_point_cloud_transform.cpp )
#target_link_libraries( test_point_cloud_transform config L3 ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY}  ${Boost_FILESYSTEM_LIBRARY})

#add_executable( test_dataset_runner test_dataset_runner.cpp )
#target_link_libraries( test_dataset_runner L3 ${Boost_LIBRARIES}  readline curses )

add_executable( test_estimator_runner test_estimator_runner.cpp )
target_link_libraries( test_estimator_runner config L3 ${Boost_LIBRARIES} )

#add_executable( test_libICP test_libICP.cpp )
#target_link_libraries( test_libICP config L3 ${Boost_LIBRARIES} )

#add_executable( test_utils test_utils.cpp )
#target_link_libraries( test_utils L3 ${Boost_LIBRARIES}  )

#add_executable( test_locale test_locale.cpp )
#target_link_libraries( test_locale config L3 ${Boost_LIBRARIES}  )

#add_executable( test_pose_homogeneous test_pose_homogeneous.cpp )
#target_link_libraries( test_pose_homogeneous config L3 ${Boost_LIBRARIES}  )

#add_executable( test_windower test_windower.cpp )
#target_link_libraries( test_windower config L3 ${Boost_LIBRARIES}  )

#add_executable( test_swathe_builder_INS test_swathe_builder_INS.cpp )
#target_link_libraries( test_swathe_builder_INS config L3 ${Boost_LIBRARIES}  )

#add_executable( test_swathe_builder_LHLV test_swathe_builder_LHLV.cpp )
#target_link_libraries( test_swathe_builder_LHLV config L3 ${Boost_LIBRARIES}  )

add_executable( test_buffered_swathe_builder_LHLV test_buffered_swathe_builder_LHLV.cpp )
target_link_libraries( test_buffered_swathe_builder_LHLV config L3 ${Boost_LIBRARIES}  )

add_executable( test_iterator test_iterator.cpp )
target_link_libraries( test_iterator config L3 ${Boost_LIBRARIES}  )

#add_executable( test_reader_bandwidth test_reader_bandwidth.cpp )
#target_link_libraries( test_reader_bandwidth config L3 ${Boost_LIBRARIES}  )

#add_executable( test_binary_reader test_binary_reader.cpp )
#target_link_libraries( test_binary_reader config L3 ${Boost_LIBRARIES}  )

#add_executable( test_pose_factory test_pose_factory.cpp )
#target_link_libraries( test_pose_factory config L3 ${Boost_LIBRARIES}  )

#add_executable( test_windower_factory test_windower_factory.cpp )
#target_link_libraries( test_windower_factory config L3 ${Boost_LIBRARIES}  )

#add_executable( test_projector test_projector.cpp )
#target_link_libraries( test_projector config L3 ${Boost_LIBRARIES}  )

#add_executable( test_smoother test_smoother.cpp )
#target_link_libraries( test_smoother config L3 ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY}  ${Boost_FILESYSTEM_LIBRARY})

#add_executable( test_chain_builder test_chain_builder.cpp )
#target_link_libraries( test_chain_builder config L3 ${Boost_LIBRARIES}  )

#add_executable( test_predictor test_predictor.cpp )
#target_link_libraries( test_predictor config L3 ${Boost_LIBRARIES}  )

#add_executable( test_library_adapters test_library_adapters.cpp )
#target_link_libraries( test_library_adapters config L3 ${Boost_LIBRARIES}  )

#add_executable( test_readers test_readers.cpp )
#target_link_libraries( test_readers config L3 ${Boost_LIBRARIES}  )

#add_executable( test_sequential_file_reader test_sequential_file_reader.cpp )
#target_link_libraries( test_sequential_file_reader config L3 ${Boost_LIBRARIES}  )

#add_executable( test_length_estimator test_length_estimator.cpp )
#target_link_libraries( test_length_estimator config L3 ${Boost_LIBRARIES}  )

#add_executable( test_experience_builder test_experience_builder.cpp )
#target_link_libraries( test_experience_builder config L3 ${Boost_LIBRARIES}  )

#add_executable( test_experience_query test_experience_query.cpp )
#target_link_libraries( test_experience_query config L3 ${Boost_LIBRARIES}  )

#add_executable( test_experience_pipeline test_experience_pipeline.cpp )
#target_link_libraries( test_experience_pipeline config L3 ${Boost_LIBRARIES}  )

#add_executable( test_runner test_runner.cpp )
#target_link_libraries( test_runner config L3 ${Boost_LIBRARIES}  )

#add_executable( test_swathe_builder_LHLV_point_cloud test_swathe_builder_LHLV_point_cloud.cpp )
#target_link_libraries( test_swathe_builder_LHLV_point_cloud config L3 ${Boost_LIBRARIES}  )

#add_executable( test_pipeline_stage_1 test_pipeline_stage_1.cpp )
#target_link_libraries( test_pipeline_stage_1 config L3 ${Boost_LIBRARIES}  )

#add_executable( test_pipeline_stage_2 test_pipeline_stage_2.cpp )
#target_link_libraries( test_pipeline_stage_2 config L3 ${Boost_LIBRARIES}  )

#add_executable( test_pipeline_stage_3 test_pipeline_stage_3.cpp )
#target_link_libraries( test_pipeline_stage_3 config L3 ${Boost_LIBRARIES}  )


#add_executable( test_histogram_smoothing test_histogram_smoothing.cpp )
#target_link_libraries( test_histogram_smoothing config L3 ${Boost_LIBRARIES}  )


