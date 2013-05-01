add_library( L3Vis 
                Visualisers.cpp 
                Controller.cpp 
                Components.cpp 
                Layouts.cpp 
                ExternalInterface.cpp
                )
target_link_libraries( L3Vis L3 GLV glut lua )

add_subdirectory( apps )
add_subdirectory( tests )
