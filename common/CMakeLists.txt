# Common: estimation
file( GLOB estimation_srcs "estimation/*.cpp" )
file( GLOB estimation_headers "estimation/*.h" )

# Common: imagery
file( GLOB imagery_srcs "imagery/*.cpp" )
file( GLOB imagery_headers "imagery/*.h" )

add_library( Common ${estimation_srcs} ${imagery_srcs} )

target_link_libraries( Common ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )
