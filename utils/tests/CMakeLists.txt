add_executable( test_lua_interface test_lua_interface.cpp )
target_link_libraries( test_lua_interface ${LUA_LIBRARIES} Interface ${Boost_LIBRARIES} )

add_executable( test_regex test_regex.cpp )
target_link_libraries( test_regex ${Boost_LIBRARIES} )

