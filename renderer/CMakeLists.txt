add_library( L3Vis 
                Visualisers.cpp 
                Controller.cpp 
                Components.cpp 
                Layouts.cpp 
                ExternalInterface.cpp
                RenderingUtils.cpp
                )
target_link_libraries( L3Vis L3 GLV lua )

add_subdirectory( apps )
add_subdirectory( tests )
