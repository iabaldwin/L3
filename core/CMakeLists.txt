add_library( L3 
    Dataset.cpp 
    Datatypes.cpp 
    AbstractFactory.cpp 
    Iterator.cpp 
    PointCloud.cpp 
    Utils.cpp 
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
    )

target_link_libraries( L3 config++ gsl PocoFoundation tbb )

add_subdirectory( tests )
add_subdirectory( apps )
