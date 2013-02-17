#include <iostream>

#include "Definitions.h"
#include "Iterator.h"
#include "Dataset.h"
#include "Utils.h"

int main()
{
    // Load dataset
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );
    assert( dataset.validate() && dataset.load() );

    // Build iterator
    std::string LIDAR_name = dataset.LIDAR_names[0];
    //std::auto_ptr< L3::ConstantTimeIterator > iterator( new L3::ConstantTimeIterator( &dataset, LIDAR_name, 100.0 ) );

    // Constant time iterator over poses
    L3::ConstantTimeIterator< L3::Pose > iterator( dataset.pose_reader, 10.0 );
    
    // Localise
    //L3::Utils::localisePoseChainToOrigin( dataset.poses );

    // Run
    L3::Tools::Timer t;
    while (true)
    {
        t.begin(); 
        iterator.update( .1 ) ;
        std::cout << t.end() << std::endl;
    } 
    
    //while( iterator.update( .1 ) )
        ////std::cout << iterator->relativeTime() << ":" << iterator->numScans() << std::endl;
        //std::cout << "hi" << std::endl;

    //std::cout << "Done" << std::endl;
}
