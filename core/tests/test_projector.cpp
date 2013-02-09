#include <iostream>

#include "Iterator.h"
#include "Dataset.h"
#include "Projector.h"

int main()
{
    // Load dataset
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );

    assert( dataset.validate() && dataset.load() );

    // Build iterator
    std::string LIDAR_name = dataset.LIDAR_names[0];

    std::auto_ptr< L3::ConstantTimeIterator > iterator( new L3::ConstantTimeIterator( &dataset, LIDAR_name, 10.0 ) );

    // Project swathe
    std::auto_ptr<L3::Projector> projector( new L3::Projector() );

    projector->project( iterator->getSwathe() );
}
