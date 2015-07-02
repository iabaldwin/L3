add_library( L3 
    Core.cpp
    Dataset.cpp 
    Datatypes.cpp 
    Iterator.cpp 
    SwatheBuilder.cpp 
    Reflectance.cpp
    Windower.cpp 
    AbstractFactory.cpp 
    PointCloud.cpp 
    Utils.cpp 
    Reader.cpp 
    Writer.cpp 
    Experience.cpp 
    SpatialData.cpp
    Projector.cpp 
    Configuration.cpp 
    Filter.cpp 
    Tracking.cpp 
    Estimator.cpp 
    Runner.cpp 
    Histogram.cpp
    Smoother.cpp 
    Simulator.cpp 
    Predictor.cpp
    PoseWindower.cpp
    ScanMatching.cpp
    VelocityProvider.cpp
    Performance.cpp
    Misc.cpp
    )

target_link_libraries( L3 ${GSL_LIBRARIES} ${TBB_LIBRARIES} ${PCL_LIBRARIES} MI lapack blas ${BAYES++_LIBRARIES} ${Poco_LIBRARIES} ${CONFIG} )

add_subdirectory( tests )
add_subdirectory( apps )

# Installation
file( GLOB L3_headers "*.h" )

install( TARGETS L3 DESTINATION lib/  )
install( FILES ${L3_headers} DESTINATION include/L3  )
