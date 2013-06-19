include_directories( "/Users/ian/code/thirdparty/dlib-18.2/" )

find_package( Freetype REQUIRED )

find_package( GLUT REQUIRED )                                    # Rendering
include_directories( ${GLUT_INCLUDE_DIR} )

find_package( OpenGL REQUIRED )                                  # Rendering
include_directories( ${OPENGL_INCLUDE_DIR} )

#find_package( Bullet REQUIRED )
#include_directories( ${BULLET_INCLUDE_DIRS} )

# 3D font experiments
#include_directories( ${FREETYPE_INCLUDE_DIRS} )
#include_directories( "/System/Library/Frameworks/OpenGL.framework/Headers/" )
#add_definitions( -DOGLFT_NO_QT -DOGLFT_NO_SOLID )

#include_directories( "/Users/ian/code/thirdparty/bullet/Demos/OpenGL/" )

add_library( L3Vis 
                RenderCore.cpp
                Visualisers.cpp 
                Components.cpp 
                Plotters.cpp 
                AlgorithmRenderer.cpp
                ViewController.cpp 
                Layouts.cpp 
                Imagery.cpp
                GLVInterface.cpp
                RenderingUtils.cpp
                CommandInterface.cpp
                LogCapture.cpp
                Widgets.cpp
                )
            
                #QueryInterface.cpp
                #debug/GLDebugDrawer.cpp 
                #target_link_libraries( L3Vis L3 GLV lua opencv_highgui opencv_core Common OGLFT Interface ${BULLET_LIBRARIES} glf )

target_link_libraries( L3Vis L3 GLV lua opencv_highgui opencv_core Common Interface pcl_search pcl_octree )

add_subdirectory( apps )
add_subdirectory( tests )
