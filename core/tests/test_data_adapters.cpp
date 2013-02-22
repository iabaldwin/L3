#include "Utils.h"
#include "Dataset.h"
#include "Iterator.h"
#include "Projector.h"
#include "PointCloud.h"
#include "SwatheBuilder.h"
#include "DataAdapters.h"

int main()
{
    /*
     *Dataset
     */
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );
    if ( !( dataset.validate() && dataset.load() ) )
        throw std::exception();

    std::string LIDAR_name = dataset.LIDAR_names[0];

    /*
     *Constant time iterator over poses
     */
    L3::ConstantTimeIterator< L3::SE3 >  pose_iterator( dataset.pose_reader, 10.0 );
    L3::ConstantTimeIterator< L3::LMS151 > LIDAR_iterator( dataset.LIDAR_readers.front(), 60.0 );
    
    double time = 1328534146.40;

    // Run for some time
    L3::SwatheBuilder swathe_builder( &pose_iterator, &LIDAR_iterator );
    for( int i=0; i<20; i++)
        assert( swathe_builder.update( time += 2 ) );

    /*
     *Projector  
     */
    L3::SE3 projection(0,0,0,.1,.2,.3);
    std::auto_ptr< L3::Projector<double> > projector( new L3::Projector<double>( &projection ) );

    /*
     *Do Projection
     */
    L3::PointCloudXYZ<double> cloud = projector->project( swathe_builder.swathe );


    L3::Utils::BEGBROKE b;
    cloud >> b;
    L3::writePCLASCII( "test.pcd", cloud );
}
