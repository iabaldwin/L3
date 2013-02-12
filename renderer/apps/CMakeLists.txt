add_executable( run_renderer run_renderer.cpp )
target_link_libraries( run_renderer GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} L3 boost_filesystem boost_system)

add_executable( plot_lhlv plot_lhlv.cpp )
target_link_libraries( plot_lhlv GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} L3 boost_filesystem boost_system)


