#include <iostream>

#include "Definitions.h"
#include "Iterator.h"
#include "Dataset.h"
#include "Utils.h"

int main()
{
    // Load dataset
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-04-16-20-05-30NightWoodstock1/");
    if ( !( dataset.validate() && dataset.load() ) )
        exit(-1);

    // Constant time iterator over poses
    L3::ConstantTimeIterator< L3::LMS151 > iterator( dataset.LIDAR_readers.begin()->second );
  
    iterator.swathe_length = 30.0;

    double time = dataset.start_time;
        
    std::cout.precision(15);

    // Run
    while (true)
    {
        usleep( .1*1e6 );
        if ( !iterator.update( time += 1 ) )
            exit(-1);
   
        std::cout << time << "-->" << iterator.window.front().first << "\t:" << iterator.window.back().first << "\t:" << iterator.window.back().first - iterator.window.front().first <<  "\t(" << iterator.window.size() << ")" << std::endl;
    } 
}
