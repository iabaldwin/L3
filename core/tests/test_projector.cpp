#include <iostream>

#include "Iterator.h"
#include "Utils.h"
#include "Dataset.h"
#include "Projector.h"

int main()
{
    // Load dataset
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );
    assert( dataset.validate() && dataset.load() );

    // Centre poses
    L3::Utils::localisePoseChainToOrigin( dataset.poses );
  
    // Build iterator
    std::string LIDAR_name = dataset.LIDAR_names[0];
    std::auto_ptr< L3::ConstantTimeIterator > iterator( new L3::ConstantTimeIterator( &dataset, LIDAR_name, 10.0 ) );


    POSE_SEQUENCE::iterator it = dataset.poses.begin();

    while( it != dataset.poses.end() )
    {
        //std::cout << *(*it) << std::endl;
        it++;
    }

    // Project swathe
    std::auto_ptr<L3::Projector> projector( new L3::Projector() );

    SWATHE* swathe = iterator->getSwathe();
    SWATHE::iterator it2 = swathe->begin();

    while( it2!=swathe->end() )
    {
        std::cout << *((*it2).first) << std::endl;
        it2++;
    }

    //L3::PointCloudXYZ<double> cloud = projector->project( iterator->getSwathe() );
    //std::cout << cloud << std::endl;
}
