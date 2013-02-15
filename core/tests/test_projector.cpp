#include <iostream>

#include "Iterator.h"
#include "Utils.h"
#include "Dataset.h"
#include "Projector.h"

int main()
{
    // Load dataset
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );
    //assert( dataset.validate() && dataset.load() );

    dataset.validate();
    dataset.load();

    // Centre poses
    L3::Utils::localisePoseChainToMean( dataset.poses );

    // Build iterator
    std::string LIDAR_name = dataset.LIDAR_names[0];
    std::auto_ptr< L3::ConstantTimeIterator > iterator( new L3::ConstantTimeIterator( &dataset, LIDAR_name, 60.0 ) );

    // Projector  
    std::auto_ptr< L3::Projector<double> > projector( new L3::Projector<double>() );

    // Do Projection, & average
    for ( int i=0; i< 1000; i++ )
    {
        L3::PointCloudXYZ<double> cloud = projector->project( iterator->getSwathe() );
    }
}