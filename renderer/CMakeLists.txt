find_package( Freetype REQUIRED )

include_directories( ${FREETYPE_INCLUDE_DIRS} )

include_directories( "/System/Library/Frameworks/OpenGL.framework/Headers/" )

add_definitions( -DOGLFT_NO_QT -DOGLFT_NO_SOLID )

add_library( L3Vis 
                Visualisers.cpp 
                Controller.cpp 
                Components.cpp 
                Layouts.cpp 
                ExternalInterface.cpp
                RenderingUtils.cpp
                TextRenderer.cpp
                )
            
            target_link_libraries( L3Vis L3 GLV lua opencv_highgui opencv_core Common OGLFT )

add_subdirectory( apps )
add_subdirectory( tests )
