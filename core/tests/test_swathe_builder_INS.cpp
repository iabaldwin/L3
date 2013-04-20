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

    // Constant time iterator over poses
    L3::ConstantTimeIterator< L3::SE3 >  pose_iterator( dataset.pose_reader );
    pose_iterator.swathe_length = 60;
    
    L3::ConstantTimeIterator< L3::LMS151 > LIDAR_iterator( dataset.LIDAR_readers.begin()->second );
    LIDAR_iterator.swathe_length = 60;
    
    double time = dataset.start_time;

    L3::ConstantTimeWindower<L3::SE3> pose_windower( &pose_iterator );
    
    L3::SwatheBuilder swathe_builder( &pose_windower, &LIDAR_iterator );

    L3::Tools::Timer t;

    // Run
    double increment = .02;
    while (true)
    {
        usleep( increment*1e6 );
        t.begin();
        swathe_builder.update( time += increment ) ;
        std::cout << __PRETTY_FUNCTION__ << ":" << t.end() << "\t(" << swathe_builder.swathe.size() << ")" << std::endl;
    } 
}
