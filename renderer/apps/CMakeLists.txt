#add_executable( run_renderer run_renderer.cpp )
#target_link_libraries( run_renderer GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

#add_executable( plot_lhlv plot_lhlv.cpp )
#target_link_libraries( plot_lhlv GLV ${GLUT_LIBRARIES} ${GLEW_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

#add_executable( plot_pose_chain plot_pose_chain.cpp )
#target_link_libraries( plot_pose_chain GLV ${GLUT_LIBRARIES} ${GLEW_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

#add_executable( plot_point_cloud plot_point_cloud.cpp )
#target_link_libraries( plot_point_cloud GLV ${GLUT_LIBRARIES} ${GLEW_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( plot_experience plot_experience.cpp )
target_link_libraries( plot_experience GLV ${GLUT_LIBRARIES} ${GLEW_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

#add_executable( plot_chain_builder plot_chain_builder.cpp )
#target_link_libraries( plot_chain_builder GLV ${GLUT_LIBRARIES} ${GLEW_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})

add_executable( L3_viz L3.cpp )
target_link_libraries( L3_viz GLV ${GLUT_LIBRARIES} ${GLEW_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis  ${Boost_LIBRARIES})


