add_library( L3 
    Dataset.cpp 
    Datatypes.cpp 
    Iterator.cpp 
    AbstractFactory.cpp 
    SwatheBuilder.cpp 
    Windower.cpp 
    PointCloud.cpp 
    Utils.cpp 
    Reader.cpp 
    Writer.cpp 
    Experience.cpp 
    Projector.cpp 
    Configuration.cpp 
    Estimator.cpp 
    Runner.cpp 
    Histogram.cpp
    Smoother.cpp 
    Integrator.cpp 
    Simulator.cpp 
    Predictor.cpp
    PoseProvider.cpp
    ScanMatching.cpp
    Misc.cpp
    )

target_link_libraries( L3 config++ gsl PocoFoundation tbb ICP pcl_common pcl_kdtree MIToolBox )

add_subdirectory( tests )
add_subdirectory( apps )
