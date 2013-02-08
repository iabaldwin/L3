add_library( L3  Reader.cpp Dataset.cpp Utils.cpp )
target_link_libraries( L3 config++ )
add_subdirectory( tests )
add_subdirectory( apps )
