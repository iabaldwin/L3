add_executable( test_app test_app.cpp )
target_link_libraries( test_app GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} L3 boost_filesystem boost_system)

add_executable( test_iterator_visual test_iterator.cpp )
target_link_libraries( test_iterator_visual GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} L3 boost_filesystem boost_system)
