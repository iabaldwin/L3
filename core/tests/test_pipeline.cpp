#include <iostream>

#include "Iterator.h"
#include "Utils.h"
#include "Dataset.h"
#include "Projector.h"
#include "SwatheBuilder.h"

int main()
{
    /*
     *Load dataset
     */
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );
    if ( !( dataset.validate() && dataset.load() ) )
        throw std::exception();

    std::string LIDAR_name = dataset.LIDAR_names[0];

    /*
     *Constant time iterator over poses
     */
    L3::ConstantTimeIterator< L3::LMS151 > LIDAR_iterator( dataset.LIDAR_readers.front(), 60.0 );
    L3::ConstantTimeIterator< L3::SE3 >  pose_iterator( dataset.pose_reader, LIDAR_iterator.swathe_length );
    
    double time = dataset.start_time;

    L3::SwatheBuilder swathe_builder( &pose_iterator, &LIDAR_iterator );

    /*
     * Projector  
     */
    L3::SE3 projection(0,0,0,.1,.2,.3);
    std::auto_ptr< L3::Projector<double> > projector( new L3::Projector<double>( &projection ) );

    /*
     * Run
     */
    L3::Tools::Timer t;
    double increment = 1.0;
    while( true )
    {
        t.begin();
        
        L3::PointCloud<double> cloud = projector->project( swathe_builder.swathe );
      
        if ( !swathe_builder.update( time += increment ))
            throw std::exception();

        double end_time = t.end(); 
        //std::cout << cloud.size() << " pts in " << end_time << "s" << "\t" << "[" << (double)cloud.size()/end_time << " pts/s" << "]" << std::endl;
        std::cout << cloud.size() << ", " << end_time << std::endl;
    }
}
