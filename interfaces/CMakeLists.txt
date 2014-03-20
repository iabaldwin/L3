find_package( PythonLibs REQUIRED )

include_directories( ${PYTHON_INCLUDE_DIRS} )
add_library( hello_ext SHARED hw.cpp )

set_target_properties(hello_ext PROPERTIES PREFIX "")
set_target_properties(hello_ext PROPERTIES OUTPUT_NAME "hello_ext.so")
set_target_properties(hello_ext PROPERTIES SUFFIX "")

target_link_libraries( hello_ext ${PYTHON_LIBRARIES} ${Boost_LIBRARIES} )
