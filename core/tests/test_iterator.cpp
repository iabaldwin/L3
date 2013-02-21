#include <iostream>

#include "Definitions.h"
#include "Iterator.h"
#include "Dataset.h"
#include "Utils.h"

int main()
{
    // Load dataset
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );
    if ( !( dataset.validate() && dataset.load() ) )
        throw std::exception();

    // Build iterator
    std::string LIDAR_name = dataset.LIDAR_names[0];

    std::cout << "Using " << LIDAR_name << std::endl;

    // Constant time iterator over poses
    //L3::ConstantTimeIterator< L3::Pose > iterator( dataset.pose_reader, 10.0 );
    L3::ConstantTimeIterator< L3::LIDAR > iterator( dataset.LIDAR_readers.front(), 10.0 );
    
    double time = 1328534146.40;
        
    std::cout.precision(15);

    // Run
    while (true)
    {
        //usleep( .2*1e6 );
        usleep( .002*1e6 );
        iterator.update( time += 1 ) ;
        
        std::cout << iterator.window.back().first << ":" << iterator.window.front().first << ":" << iterator.window.back().first - iterator.window.front().first << std::endl;
    } 
}
