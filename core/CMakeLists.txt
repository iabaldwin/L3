add_library( L3 
    Core.cpp
    Dataset.cpp 
    Datatypes.cpp 
    Iterator.cpp 
    SwatheBuilder.cpp 
    Windower.cpp 
    AbstractFactory.cpp 
    PointCloud.cpp 
    Utils.cpp 
    Reader.cpp 
    Writer.cpp 
    Experience.cpp 
    Projector.cpp 
    Configuration.cpp 
    Filter.cpp 
    Tracking.cpp 
    Estimator.cpp 
    Runner.cpp 
    Histogram.cpp
    Smoother.cpp 
    Integrator.cpp 
    Simulator.cpp 
    Predictor.cpp
    PoseWindower.cpp
    ScanMatching.cpp
    VelocityProvider.cpp
    Misc.cpp
    )

target_link_libraries( L3 config++ gsl PocoFoundation tbb ICP pcl_common pcl_kdtree pcl_search pcl_features MIToolBox fftw3 lapack blas itpp_static )

add_subdirectory( tests )
add_subdirectory( apps )
