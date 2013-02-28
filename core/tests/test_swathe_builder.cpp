#include <iostream>

#include "Definitions.h"
#include "Iterator.h"
#include "Dataset.h"
#include "Utils.h"
#include "SwatheBuilder.h"

int main()
{
    // Load dataset
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );
    
    if ( !( dataset.validate() && dataset.load() ) )
        throw std::exception();

    std::string LIDAR_name = dataset.LIDAR_names[0];

    // Constant time iterator over poses
    L3::ConstantTimeIterator< L3::SE3 >  pose_iterator( dataset.pose_reader, 60.0 );
    L3::ConstantTimeIterator< L3::LMS151 > LIDAR_iterator( dataset.LIDAR_readers.front(), 10.0 );
    
    double time = dataset.start_time;

    L3::SwatheBuilder swathe_builder( &pose_iterator, &LIDAR_iterator );

    L3::Tools::Timer t;

    // Run
    double increment = .02;
    while (true)
    {
        usleep( increment*1e6 );
        t.begin();
        swathe_builder.update( time += increment ) ;
        std::cout << __PRETTY_FUNCTION__ << ":" << t.end() << std::endl;
    } 
}
