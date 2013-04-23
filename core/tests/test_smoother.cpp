#include "L3.h"
#include "Smoother.h"

#include <boost/random.hpp>

int main()
{
    L3::PointCloud<double> cloud;
    cloud.points = new L3::Point<double>[10000];
    cloud.num_points = 10000;
   
    L3::gaussianCloud( &cloud, 5.0 );
  
    L3::translate( &cloud, new L3::SE3( -10, 20, 0, 0, 0, 0 ) );

    boost::shared_ptr< L3::Histogram<double> > histogram( new L3::Histogram<double>() );
    histogram->create(0, -40, 40, 0, -40, 40, 40, 40 );

    /*
     *Make histogram
     */
    histogram->operator()( &cloud );

    /*
     *Smooth
     */

    std::ofstream stream( "hist.original" );
    //std::cout << *histogram << std::endl;
    stream << *histogram;
    stream.close();

    L3::Smoother<double,5>* smoother = new L3::BoxSmoother<double,5>();

    L3::Timing::SysTimer t;
    t.begin();
    smoother->smooth( histogram.get() );
    std::cout << t.elapsed() << std::endl;

    stream.open( "hist.new" );
    stream << *histogram;
    stream.close();

    stream.open( "point_cloud" );
    stream << cloud ;
    stream.close();
}
