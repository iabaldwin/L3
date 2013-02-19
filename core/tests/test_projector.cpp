#include <iostream>

#include "Iterator.h"
#include "Utils.h"
#include "Dataset.h"
#include "Projector.h"
#include "SwatheBuilder.h"

int main()
{
    // Load dataset
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );
    if ( !( dataset.validate() && dataset.load() ) )
        throw std::exception();

    std::string LIDAR_name = dataset.LIDAR_names[0];

    // Constant time iterator over poses
    L3::ConstantTimeIterator< L3::Pose >  pose_iterator( dataset.pose_reader, 60.0 );
    L3::ConstantTimeIterator< L3::LIDAR > LIDAR_iterator( dataset.LIDAR_readers.front(), 60.0 );
    
    double time = 1328534146.40;

    L3::SwatheBuilder swathe_builder( &pose_iterator, &LIDAR_iterator );

    // Projector  
    std::auto_ptr< L3::Projector<double> > projector( new L3::Projector<double>() );

    // Run until we get a good window
    swathe_builder.update( time + 20 );

    // Do Projection, & average
    L3::Tools::Timer t;
    for ( int i=0; i< 1000; i++ )
    {
        t.begin();
        L3::PointCloudXYZ<double> cloud = projector->project( swathe_builder.swathe );
        std::cout << cloud.size() << " pts in " << t.end() << "s" << std::endl;
    }
}
