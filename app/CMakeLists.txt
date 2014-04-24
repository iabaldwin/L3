##==========
## App
##==========

find_package( GLUT REQUIRED )                                    # Rendering
include_directories( ${GLUT_INCLUDE_DIR} )

find_package( OpenGL REQUIRED )                                  # Rendering
include_directories( ${OPENGL_INCLUDE_DIR} )

add_executable( L3app app.cpp )
target_link_libraries( L3app ${GLV_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis ${Boost_LIBRARIES} ${Poco_LIBRARIES} ${CONFIG} )
