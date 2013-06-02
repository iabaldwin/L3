#include <iostream>

#include "Definitions.h"
#include "Iterator.h"
#include "Dataset.h"
#include "Utils.h"
#include "SwatheBuilder.h"
#include "Configuration.h"

int main()
{
    // Load dataset
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-04-16-20-05-30NightWoodstock1/" ) ;
    
    if ( !( dataset.validate() && dataset.load() ) )
        exit( -1 );
    
    L3::Configuration::Mission mission( dataset );

    // Constant time iterator over poses
    L3::ConstantTimeIterator< L3::LHLV >    LHLV_iterator( dataset.LHLV_reader );
    L3::ConstantTimeIterator< L3::LMS151 >  LIDAR_iterator( dataset.LIDAR_readers[ mission.declined ] );
    
    LIDAR_iterator.swathe_length = 60;
    LHLV_iterator.swathe_length = 60;
   
    double time = dataset.start_time;

    L3::ConstantDistanceWindower pose_windower( &LHLV_iterator, 60 );
    L3::BufferedSwatheBuilder   swathe_builder( &pose_windower, &LIDAR_iterator );

    L3::Timing::SysTimer t;

    // Run
    double increment = .02;
    while (true)
    {
        usleep( increment*1e6 );
        t.begin();
        
        time += increment; 

        LHLV_iterator.update( time );
        LIDAR_iterator.update( time );
        pose_windower.update( time );

        swathe_builder.update( time );
        std::cout << __PRETTY_FUNCTION__ << ":" << t.elapsed() << "\t(" << swathe_builder.swathe.size() << ")" << std::endl;
    } 
}
