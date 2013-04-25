add_library( L3Vis Visualisers.cpp Controller.cpp Components.cpp )
target_link_libraries( L3Vis L3 GLV glut )

add_subdirectory( apps )
add_subdirectory( tests )
