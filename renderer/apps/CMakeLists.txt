add_executable( run_renderer run_renderer.cpp )
target_link_libraries( run_renderer GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} L3  ${Boost_LIBRARIES})

add_executable( plot_lhlv plot_lhlv.cpp )
target_link_libraries( plot_lhlv GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} L3  ${Boost_LIBRARIES})

add_executable( plot_pose_chain plot_pose_chain.cpp )
target_link_libraries( plot_pose_chain GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} L3  ${Boost_LIBRARIES})

add_executable( plot_point_cloud plot_point_cloud.cpp )
target_link_libraries( plot_point_cloud GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} L3  ${Boost_LIBRARIES})

