add_executable( test_input test_input.cpp )
target_link_libraries( test_input GLV ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES}  ${GLUT_LIBRARIES} ${Boost_LIBRARIES})

add_executable( test_component test_component.cpp )
target_link_libraries( test_component GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_interface test_interface.cpp )
target_link_libraries( test_interface GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_composite test_composite.cpp )
target_link_libraries( test_composite GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_controller test_controller.cpp )
target_link_libraries( test_controller GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_imagery test_imagery.cpp )
target_link_libraries( test_imagery GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_iterator_visual test_iterator.cpp )
target_link_libraries( test_iterator_visual GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_point_clouds_transform test_point_clouds_transform.cpp )
target_link_libraries( test_point_clouds_transform GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_swathe_builder_INS_visual test_swathe_builder_INS.cpp )
target_link_libraries( test_swathe_builder_INS_visual GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_swathe_builder_LHLV_visual test_swathe_builder_LHLV.cpp )
target_link_libraries( test_swathe_builder_LHLV_visual GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_renderer_pipeline test_renderer_pipeline.cpp )
target_link_libraries( test_renderer_pipeline GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_draw_cube test_draw_cube.cpp )
target_link_libraries( test_draw_cube GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_histogram_renderer test_histogram_renderer.cpp )
target_link_libraries( test_histogram_renderer GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_histogram_smoothing_visual test_histogram_smoothing.cpp )
target_link_libraries( test_histogram_smoothing_visual GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_smoother_visual test_smoother.cpp )
target_link_libraries( test_smoother_visual GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_experience_renderer test_experience_renderer.cpp )
target_link_libraries( test_experience_renderer GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_layout test_layout.cpp )
target_link_libraries( test_layout GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_dataset_layout test_dataset_layout.cpp )
target_link_libraries( test_dataset_layout GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_estimator_layout test_estimator_layout.cpp )
target_link_libraries( test_estimator_layout GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_algorithm_visual test_algorithm.cpp )
target_link_libraries( test_algorithm_visual GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_pose_windower test_pose_windower.cpp )
target_link_libraries( test_pose_windower GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_scan_projection test_scan_projection.cpp )
target_link_libraries( test_scan_projection GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_kinematics test_kinematics.cpp )
target_link_libraries( test_kinematics GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_integrator test_integrator.cpp )
target_link_libraries( test_integrator GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_pose_prediction test_pose_prediction.cpp )
target_link_libraries( test_pose_prediction GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_estimator_visual test_estimator.cpp )
target_link_libraries( test_estimator_visual GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_back_integrator test_back_integrator.cpp )
target_link_libraries( test_back_integrator GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_estimator_prediction test_estimator_prediction.cpp )
target_link_libraries( test_estimator_prediction GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

#==========
# Pipelines
#==========
add_executable( test_pipeline_stage_1_visual test_pipeline_stage_1.cpp )
target_link_libraries( test_pipeline_stage_1_visual GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_pipeline_stage_2_visual test_pipeline_stage_2.cpp )
target_link_libraries( test_pipeline_stage_2_visual GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_pipeline_stage_3_visual test_pipeline_stage_3.cpp )
target_link_libraries( test_pipeline_stage_3_visual GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_pipeline_stage_4_visual test_pipeline_stage_4.cpp )
target_link_libraries( test_pipeline_stage_4_visual GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( test_pipeline_stage_5_visual test_pipeline_stage_5.cpp )
target_link_libraries( test_pipeline_stage_5_visual GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})



