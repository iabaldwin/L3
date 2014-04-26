find_package( Freetype REQUIRED )

find_package( GLUT REQUIRED )                                    # Rendering
include_directories( ${GLUT_INCLUDE_DIR} )

find_package( OpenGL REQUIRED )                                  # Rendering
include_directories( ${OPENGL_INCLUDE_DIR} )

find_package( GLV REQUIRED )
include_directories( ${GLV_INCLUDE_DIR} )

find_package( Lua51 REQUIRED )
include_directories( ${LUA_INCLUDE_DIR} )

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
          
target_link_libraries( L3Vis L3 ${GLV_LIBRARY} ${LUA_LIBRARIES} ${OpenCV_LIBRARIES} Common Interface ${PCL_LIBRARIES} )

add_subdirectory( apps )
add_subdirectory( tests )
