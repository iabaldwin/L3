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

    // Constant time iterator over poses
    L3::ConstantTimeIterator< L3::LMS151 > iterator( dataset.LIDAR_readers.front() );
  
    iterator.swathe_length = 30.0;

    double time = dataset.start_time;
        
    std::cout.precision(15);

    // Run
    while (true)
    {
        usleep( .1*1e6 );
        if ( !iterator.update( time += 1 ) )
            throw std::exception();
    
        std::cout << time << "-->" << iterator.window.front().first << "\t:" << iterator.window.back().first << "\t:" << iterator.window.back().first - iterator.window.front().first <<  "\t(" << iterator.window.size() << ")" << std::endl;
    } 
}
