find_package( Freetype REQUIRED )

find_package( GLUT REQUIRED )                                    # Rendering
include_directories( ${GLUT_INCLUDE_DIR} )

find_package( OpenGL REQUIRED )                                  # Rendering
include_directories( ${OPENGL_INCLUDE_DIR} )

find_package( Bullet REQUIRED )
include_directories( ${BULLET_INCLUDE_DIRS} )

# 3D font experiments
include_directories( ${FREETYPE_INCLUDE_DIRS} )
include_directories( "/System/Library/Frameworks/OpenGL.framework/Headers/" )
add_definitions( -DOGLFT_NO_QT -DOGLFT_NO_SOLID )

add_library( L3Vis 
                Visualisers.cpp 
                Controller.cpp 
                Components.cpp 
                Layouts.cpp 
                TextRenderer.cpp
                GLVInterface.cpp
                RenderingUtils.cpp
                CommandInterface.cpp
                )
            
target_link_libraries( L3Vis L3 GLV lua opencv_highgui opencv_core Common OGLFT Interface ${BULLET_LIBRARIES} glf )

add_subdirectory( apps )
add_subdirectory( tests )
