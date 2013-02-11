#include <iostream>

#include "Definitions.h"
#include "Iterator.h"
#include "Dataset.h"
#include "Utils.h"

int main()
{
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );

    assert( dataset.validate() && dataset.load() );

    std::string LIDAR_name = dataset.LIDAR_names[0];

    std::auto_ptr< L3::ConstantTimeIterator > iterator( new L3::ConstantTimeIterator( &dataset, LIDAR_name, 100.0 ) );

    L3::Utils::localisePoseChainToOrigin( dataset.poses );

    SWATHE* s = iterator->getSwathe();

    SWATHE::iterator it = s->begin();

    std::cout.precision(12);

    //while( it != s->end() )
    //{
        ////std::cout << *((*it).first) << std::endl;
        ////std::cout << *((*it).second) << std::endl;
        //it++;
    //}

    while( iterator->update( .1 ) )
        std::cout << iterator->relativeTime() << ":" << iterator->numScans() << std::endl;

    std::cout << "Done" << std::endl;
}
