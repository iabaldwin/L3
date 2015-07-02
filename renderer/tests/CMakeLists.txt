#add_executable( test_itpp_visual test_itpp.cpp )
#target_link_libraries( test_itpp_visual  ${GLV_LIBRARY} L3Vis L3 ${Boost_LIBRARIES}  ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES}  ${GLUT_LIBRARIES} fftw3 lapack blas itpp_static ${CONFIG} )

#add_executable( test_ray_query test_ray_query.cpp )
#target_link_libraries( test_ray_query ${GLV_LIBRARY} L3Vis ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES}  ${GLUT_LIBRARIES} ${Boost_LIBRARIES} ${FREETYPE_LIBRARIES} ${BULLET_LIBRARIES} ${CONFIG} )

#add_executable( test_freetype test_freetype.cpp )
#target_link_libraries( test_freetype ${GLV_LIBRARY} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES}  ${GLUT_LIBRARIES} ${Boost_LIBRARIES} ${FREETYPE_LIBRARIES} ${CONFIG} )

#add_executable( test_coordinate_transforms test_coordinate_transforms.cpp )
#target_link_libraries( test_coordinate_transforms ${GLV_LIBRARY} L3Vis ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES}  ${GLUT_LIBRARIES} ${Boost_LIBRARIES} ${FREETYPE_LIBRARIES} glf ${CONFIG} )

#add_executable( test_libglf test_libglf.cpp )
#target_link_libraries( test_libglf ${GLV_LIBRARY} L3Vis ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES}  ${GLUT_LIBRARIES} ${Boost_LIBRARIES} ${FREETYPE_LIBRARIES} glf ${CONFIG} )

#add_executable( test_controls test_controls.cpp )
#target_link_libraries( test_controls ${GLV_LIBRARY} L3Vis ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES}  ${GLUT_LIBRARIES} ${Boost_LIBRARIES} ${CONFIG} )

add_executable( test_input test_input.cpp )
target_link_libraries( test_input ${GLV_LIBRARY} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES}  ${GLUT_LIBRARIES} ${Boost_LIBRARIES} ${CONFIG} )

#add_executable( test_spatial_grid test_spatial_grid.cpp )
#target_link_libraries( test_spatial_grid ${GLV_LIBRARY} L3Vis ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES}  ${GLUT_LIBRARIES} ${Boost_LIBRARIES} ${CONFIG} )

#add_executable( test_component test_component.cpp )
#target_link_libraries( test_component ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${CONFIG} )

#add_executable( test_text_renderer test_text_renderer.cpp )
#target_link_libraries( test_text_renderer ${GLV_LIBRARY} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES}  ${GLUT_LIBRARIES} L3 L3Vis ${Boost_LIBRARIES} ${FREETYPE_LIBRARIES} FTGL  ${CONFIG} )

#add_executable( test_interface test_interface.cpp )
#target_link_libraries( test_interface ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${CONFIG} )

#add_executable( test_composite test_composite.cpp )
#target_link_libraries( test_composite ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${CONFIG} )

add_executable( test_controller test_controller.cpp )
target_link_libraries( test_controller ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${Poco_LIBRARIES}  ${CONFIG} )

#add_executable( test_imagery test_imagery.cpp )
#target_link_libraries( test_imagery ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${CONFIG} )

add_executable( test_iterator_visual test_iterator.cpp )
target_link_libraries( test_iterator_visual ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${Poco_LIBRARIES}  ${CONFIG} )

add_executable( test_swathe_builder_INS_visual test_swathe_builder_INS.cpp )
target_link_libraries( test_swathe_builder_INS_visual ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${Poco_LIBRARIES}  ${CONFIG} )

#add_executable( test_swathe_builder_LHLV_visual test_swathe_builder_LHLV.cpp )
#target_link_libraries( test_swathe_builder_LHLV_visual ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${CONFIG} )

#add_executable( test_swathe_builder_LHLV_constant_distance_visual test_swathe_builder_LHLV_constant_distance.cpp )
#target_link_libraries( test_swathe_builder_LHLV_constant_distance_visual ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${CONFIG} )

#add_executable( test_point_clouds_transform test_point_clouds_transform.cpp )
#target_link_libraries( test_point_clouds_transform ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${CONFIG} )

add_executable( test_draw_cube test_draw_cube.cpp )
target_link_libraries( test_draw_cube ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${Poco_LIBRARIES}  ${CONFIG} )

#add_executable( test_histogram_renderer test_histogram_renderer.cpp )
#target_link_libraries( test_histogram_renderer ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${CONFIG} )

#add_executable( test_histogram_smoothing_visual test_histogram_smoothing.cpp )
#target_link_libraries( test_histogram_smoothing_visual ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${CONFIG} )

#add_executable( test_smoother_visual test_smoother.cpp )
#target_link_libraries( test_smoother_visual ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${CONFIG} )

#add_executable( test_experience_renderer test_experience_renderer.cpp )
#target_link_libraries( test_experience_renderer ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${CONFIG} )

add_executable( test_reflectance_renderer test_reflectance_renderer.cpp )
target_link_libraries( test_reflectance_renderer ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${Poco_LIBRARIES}  ${CONFIG} )

#add_executable( test_layout test_layout.cpp )
#target_link_libraries( test_layout ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${CONFIG} )

add_executable( test_dataset_layout test_dataset_layout.cpp )
target_link_libraries( test_dataset_layout ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${Poco_LIBRARIES}  ${CONFIG} )

add_executable( test_estimator_layout test_estimator_layout.cpp )
target_link_libraries( test_estimator_layout ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} readline curses  ${Poco_LIBRARIES}  ${CONFIG} )

#add_executable( test_algorithm_visual test_algorithm.cpp )
#target_link_libraries( test_algorithm_visual ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${CONFIG} )

#add_executable( test_pose_windower test_pose_windower.cpp )
#target_link_libraries( test_pose_windower ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${CONFIG} )

#add_executable( test_scan_projection test_scan_projection.cpp )
#target_link_libraries( test_scan_projection ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${CONFIG} )

#add_executable( test_kinematics test_kinematics.cpp )
#target_link_libraries( test_kinematics ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${CONFIG} )

#add_executable( test_integrator test_integrator.cpp )
#target_link_libraries( test_integrator ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${CONFIG} )

#add_executable( test_pose_prediction test_pose_prediction.cpp )
#target_link_libraries( test_pose_prediction ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${CONFIG} )

#add_executable( test_estimator_visual test_estimator.cpp )
#target_link_libraries( test_estimator_visual ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${CONFIG} )

#add_executable( test_back_integrator test_back_integrator.cpp )
#target_link_libraries( test_back_integrator ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${CONFIG} )

#add_executable( test_estimator_prediction test_estimator_prediction.cpp )
#target_link_libraries( test_estimator_prediction ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${CONFIG} )

add_executable( test_camera_transform test_camera_transform.cpp )
target_link_libraries( test_camera_transform ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES} ${Poco_LIBRARIES} ${CONFIG} )

add_executable( test_L3 test_L3.cpp )
#target_link_libraries( test_L3 ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 ${PCL_LIBRARIES} L3Vis  ${Boost_LIBRARIES} ${Poco_LIBRARIES} ${CONFIG} )
target_link_libraries( test_L3 ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 ${PCL_LIBRARIES} L3Vis  ${Boost_LIBRARIES} ${Poco_LIBRARIES} )

