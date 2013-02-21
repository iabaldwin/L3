add_executable( test_app test_app.cpp )
target_link_libraries( test_app GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} L3 boost_filesystem boost_system)

add_executable( test_iterator_visual test_iterator.cpp )
target_link_libraries( test_iterator_visual GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} L3 boost_filesystem boost_system)

add_executable( test_point_clouds test_point_clouds.cpp )
target_link_libraries( test_point_clouds GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} L3 boost_filesystem boost_system)

add_executable( test_adapters test_adapters.cpp )
target_link_libraries( test_adapters GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} L3 boost_filesystem boost_system)

add_executable( test_swathe_builder_visual test_swathe_builder.cpp )
target_link_libraries( test_swathe_builder_visual GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} L3 boost_filesystem boost_system)


