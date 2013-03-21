add_library( L3Vis Visualisers.cpp Controller.cpp )
target_link_libraries( L3Vis L3 GLV )

add_subdirectory( apps )
add_subdirectory( tests )
