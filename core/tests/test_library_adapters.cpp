#include "L3.h"

int main()
{
    /*
     *Dataset
     */
    //L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );           // Weird data with this one
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-08-09-36-42-WOODSTOCK-SLOW/" );
    if ( !( dataset.validate() && dataset.load() ) )
        throw std::exception();

    /*
     *Constant time iterator over poses
     */
    L3::ConstantTimeIterator< L3::LMS151 > LIDAR_iterator( dataset.LIDAR_readers.front(), 30.0 );
    L3::ConstantTimeIterator< L3::SE3 >  pose_iterator( dataset.pose_reader, LIDAR_iterator.swathe_length );
    
    double time = dataset.start_time;

    // Run for some time
    L3::SwatheBuilder swathe_builder( &pose_iterator, &LIDAR_iterator );
   
    // Go X seconds in
    for( int i=0; i<40; i++ )
    {
        usleep( .02*1e6 );
        swathe_builder.update( time += 2 );
    }

    /*
     *Projector  
     */
    L3::SE3 projection(0, 0, 0, 1.57, 0, 0);
    L3::PointCloud<double>* cloud = new L3::PointCloud<double>();
    std::auto_ptr< L3::Projector<double> > projector( new L3::Projector<double>( &projection, cloud ) );

    /*
     *Do Projection
     */
    projector->project( swathe_builder.swathe );

    L3::centerPointCloud( cloud );

    L3::writePCLASCII( "test.pcd", *cloud );
}