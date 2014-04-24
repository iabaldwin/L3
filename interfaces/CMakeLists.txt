find_package( PythonLibs REQUIRED )

include_directories( ${PYTHON_INCLUDE_DIRS} )
add_library( Core SHARED Core.cpp )

set_target_properties(Core PROPERTIES PREFIX "")
set_target_properties(Core PROPERTIES OUTPUT_NAME "Core.so")
set_target_properties(Core PROPERTIES SUFFIX "")

target_link_libraries( Core ${PYTHON_LIBRARIES} ${Boost_LIBRARIES} ${Poco_LIBRARIES} L3 )
