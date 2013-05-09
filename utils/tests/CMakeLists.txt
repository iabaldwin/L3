add_executable( test_lua_interface test_lua_interface.cpp )
target_link_libraries( test_lua_interface lua Interface boost_regex )

add_executable( test_regex test_regex.cpp )
target_link_libraries( test_regex boost_regex )

