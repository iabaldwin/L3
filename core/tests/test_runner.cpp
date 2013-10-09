#include <iostream>

#include "Definitions.h"
#include "Iterator.h"
#include "Dataset.h"
#include "Utils.h"
#include "SwatheBuilder.h"
#include "Runner.h"

int main( int argc, char* argv[] )
{

    if ( argc < 2 )
        return -1;

    // Load dataset
    L3::Dataset dataset( argv[1] );
    
    if ( !( dataset.validate() && dataset.load() ) )
        throw std::exception();

    // Constant time iterator over poses
    L3::ConstantTimeIterator< L3::SE3 >  pose_iterator( dataset.pose_reader );
    pose_iterator.swathe_length = 60;
    
    L3::ConstantTimeIterator< L3::LMS151 > LIDAR_iterator( dataset.LIDAR_readers.begin()->second );
    LIDAR_iterator.swathe_length = 60;
   
    L3::ConstantTimeWindower<L3::SE3> pose_windower( &pose_iterator );
    
    //L3::ThreadedTemporalRunner runner;

    //runner << &swathe_builder;

    //runner.start( dataset.start_time );

    //Run
    //double increment = .02;
    //while (true)
    //{
        //usleep( increment*1e6 );
        //t.begin();
        //swathe_builder.update( time += increment ) ;
        //std::cout << __PRETTY_FUNCTION__ << ":" << t.end() << "\t(" << swathe_builder.swathe.size() << ")" << std::endl;
    //} 
}
