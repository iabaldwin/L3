add_executable( test_iterator_visual test_iterator.cpp )
target_link_libraries( test_iterator_visual GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_point_clouds test_point_clouds.cpp )
target_link_libraries( test_point_clouds GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_point_clouds_transform test_point_clouds_transform.cpp )
target_link_libraries( test_point_clouds_transform GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_adapters test_adapters.cpp )
target_link_libraries( test_adapters GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_swathe_builder_INS_visual test_swathe_builder_INS.cpp )
target_link_libraries( test_swathe_builder_INS_visual GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_swathe_builder_LHLV_visual test_swathe_builder_LHLV.cpp )
target_link_libraries( test_swathe_builder_LHLV_visual GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_renderer_pipeline test_renderer_pipeline.cpp )
target_link_libraries( test_renderer_pipeline GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_component test_component.cpp )
target_link_libraries( test_component GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_composite test_composite.cpp )
target_link_libraries( test_composite GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_histogram_renderer test_histogram_renderer.cpp )
target_link_libraries( test_histogram_renderer GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_experience_renderer test_experience_renderer.cpp )
target_link_libraries( test_experience_renderer GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_pipeline_stage_1_visual test_pipeline_stage_1.cpp )
target_link_libraries( test_pipeline_stage_1_visual GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_data_providers test_data_providers.cpp )
target_link_libraries( test_data_providers GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_layout test_layout.cpp )
target_link_libraries( test_layout GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_dataset_layout test_dataset_layout.cpp )
target_link_libraries( test_dataset_layout GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_pose_windower test_pose_windower.cpp )
target_link_libraries( test_pose_windower GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_scan_projection test_scan_projection.cpp )
target_link_libraries( test_scan_projection GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

