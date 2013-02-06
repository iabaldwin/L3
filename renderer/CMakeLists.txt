add_executable( renderer renderer.cpp )
target_link_libraries( renderer GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})
