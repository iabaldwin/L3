add_executable( test_app test_app.cpp )
target_link_libraries( test_app GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} L3 boost_filesystem boost_system)
