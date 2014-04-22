add_executable( experience_builder experience_builder.cpp )
target_link_libraries( experience_builder ${CONFIG} L3 ${Boost_LIBRARIES} ${Poco_LIBRARIES} )

add_executable( reflectance_builder reflectance_builder.cpp )
target_link_libraries( reflectance_builder ${CONFIG} L3 ${Boost_LIBRARIES} ${Poco_LIBRARIES} )

add_executable( LHLV_filter LHLV_filter.cpp )
target_link_libraries( LHLV_filter ${CONFIG} L3 ${Boost_LIBRARIES}  ${Poco_LIBRARIES} )

add_executable( bulk_runner bulk_runner.cpp )
target_link_libraries( bulk_runner ${CONFIG} L3 ${Boost_LIBRARIES}  ${Poco_LIBRARIES} )

