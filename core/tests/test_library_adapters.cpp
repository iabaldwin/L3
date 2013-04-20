#include "L3.h"

int main()
{
    /*
     *Dataset
     */
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-08-09-36-42-WOODSTOCK-SLOW/" );
    if ( !( dataset.validate() && dataset.load() ) )
        throw std::exception();

    /*
     *Constant time iterator over poses
     */
    L3::ConstantTimeIterator< L3::LMS151 > LIDAR_iterator( dataset.LIDAR_readers.begin()->second );
    L3::ConstantTimeIterator< L3::SE3 >  pose_iterator( dataset.pose_reader );
  
    pose_iterator.swathe_length = LIDAR_iterator.swathe_length;

    double time = dataset.start_time;

    L3::ConstantTimeWindower<L3::SE3> pose_windower( &pose_iterator );

    // Run for some time
    //L3::SwatheBuilder swathe_builder( &pose_iterator, &LIDAR_iterator );
    L3::SwatheBuilder swathe_builder( &pose_windower, &LIDAR_iterator );
   
    // Go X seconds in
    for( int i=0; i<40; i++ )
    {
        usleep( .02*1e6 );
        swathe_builder.update( time += 2 );
    }

    /*
     *Projector  
     */
    L3::Configuration::Mission mission(  dataset.name() );
  
    std::cout << mission << std::endl;

    L3::SE3 calibration = L3::SE3::ZERO();
    convert( (++mission.lidars.begin())->second, calibration );

    L3::PointCloud<double>* cloud = new L3::PointCloud<double>();
    std::auto_ptr< L3::Projector<double> > projector( new L3::Projector<double>( &calibration, cloud ) );

    /*
     *Do Projection
     */
    projector->project( swathe_builder.swathe );

    //L3::centerPointCloud( cloud );

    L3::writePCLASCII( "test.pcd", *cloud );

    std::cout << "Done" << std::endl;
}
