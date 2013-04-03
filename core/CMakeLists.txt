add_library( L3 
    Dataset.cpp 
    Datatypes.cpp 
    AbstractFactory.cpp 
    Iterator.cpp 
    PointCloud.cpp 
    Utils.cpp 
    Writer.cpp 
    Experience.cpp 
    ChainBuilder.cpp 
    Configuration.cpp 
    Estimator.cpp 
    Runner.cpp )

target_link_libraries( L3 config++ gsl PocoFoundation gslcblas )

add_subdirectory( tests )
add_subdirectory( apps )
