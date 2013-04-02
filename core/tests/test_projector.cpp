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
    L3::ConstantTimeIterator< L3::SE3 >  pose_iterator( dataset.pose_reader );
    pose_iterator.swathe_length = 60.0;
    
    L3::ConstantTimeIterator< L3::LMS151 > LIDAR_iterator( dataset.LIDAR_readers.begin()->second );
    LIDAR_iterator.swathe_length = 60.0;

    /*
     *Pose provider
     */
    L3::ConstantTimePoseWindower pose_windower( &pose_iterator );
   
    /*
     *Swathe builder
     */
    L3::SwatheBuilder swathe_builder( &pose_windower, &LIDAR_iterator );


    /*
     *Projector  
     */
    L3::SE3 projection(0,0,0,.1,.2,.3);
    L3::PointCloud<double>* point_cloud = new L3::PointCloud<double>();
    std::auto_ptr< L3::Projector<double> > projector( new L3::Projector<double>( &projection, point_cloud ) );

    swathe_builder.update( dataset.start_time + 20 );

    /*
     *Do Projection, & average
     */
    L3::Tools::Timer t;
    for ( int i=0; i< 1000; i++ )
    {
        t.begin();
        projector->project( swathe_builder.swathe );
        double end_time = t.end(); 
        std::cout << point_cloud->num_points << " pts in " << end_time << "s" << std::endl;
        std::cout << (double)point_cloud->num_points/end_time << " pts/s" << std::endl;
    
    }
}
