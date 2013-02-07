add_executable( test_writer test_writer.cpp )
target_link_libraries( test_writer L3 )

add_executable( test_dataset test_dataset.cpp )
target_link_libraries( test_dataset L3 boost_system boost_filesystem )

add_executable( test_locale test_locale.cpp )
target_link_libraries( test_locale  config L3 boost_system boost_filesystem )

