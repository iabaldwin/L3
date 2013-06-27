##==========
## App
##==========

include_directories( "/Users/ian/code/thirdparty/dlib-18.2/" )

find_package( GLUT REQUIRED )                                    # Rendering
include_directories( ${GLUT_INCLUDE_DIR} )

find_package( OpenGL REQUIRED )                                  # Rendering
include_directories( ${OPENGL_INCLUDE_DIR} )


add_executable( app app.cpp )
target_link_libraries( app GLV ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} L3 L3Vis ${Boost_LIBRARIES} pcl_octree flann_cpp )
