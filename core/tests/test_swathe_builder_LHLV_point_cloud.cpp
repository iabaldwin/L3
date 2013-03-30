#include <iostream>

#include "Definitions.h"
#include "Iterator.h"
#include "Dataset.h"
#include "Utils.h"
#include "SwatheBuilder.h"
#include "ChainBuilder.h"
#include "PointCloud.h"
#include "Configuration.h"
#include "Projector.h"
#include "LibraryAdapters.h"

int main()
{
    // Load dataset
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" );
    //L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );
    
    if ( !( dataset.validate() && dataset.load() ) )
        throw std::exception();

    // Constant time iterator over poses
    L3::ConstantTimeIterator< L3::LHLV >    lhlv_iterator( dataset.LHLV_reader );

    L3::ConstantTimeIterator< L3::LMS151 >  LIDAR_iterator( dataset.LIDAR_readers.back() );
    LIDAR_iterator.swathe_length = 60;
    lhlv_iterator.swathe_length = 60;
   
    L3::ChainBuilder pose_windower( &lhlv_iterator );
   
    L3::SwatheBuilder swathe_builder( &pose_windower, &LIDAR_iterator );

    L3::PointCloud<double>* cloud = new L3::PointCloud<double>();

    L3::Configuration::Mission mission( dataset.name() );

    L3::SE3 calibration = L3::SE3::ZERO();

    L3::Configuration::convert( ((++mission.lidars.begin())->second), calibration );
    
    L3::Projector<double> projector( &calibration, cloud );

    L3::Tools::Timer t;

    // Run
    double time = dataset.start_time;

    double increment = 1;
  
    int counter = 0;

    while (true)
    {
        t.begin();
        swathe_builder.update( time += increment ) ;
        projector.project( swathe_builder.swathe );
        std::cout << __PRETTY_FUNCTION__ << ":" << t.end() << "\t(" << swathe_builder.swathe.size() << ")" << std::endl;

        if (counter++ > 180 )
            break;
    } 

    L3::writePCLASCII( "test.pcd", *cloud);

    std::cout << calibration << std::endl;
}
