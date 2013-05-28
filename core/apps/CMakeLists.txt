add_executable( experience_builder experience_builder.cpp )
target_link_libraries( experience_builder config L3 ${Boost_LIBRARIES}  )

add_executable( LHLV_extractor LHLV_extractor.cpp )
target_link_libraries( LHLV_extractor config L3 ${Boost_LIBRARIES}  )


