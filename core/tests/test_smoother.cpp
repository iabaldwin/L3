#include "L3.h"
#include "Smoother.h"

#include <boost/random.hpp>

int main()
{
    L3::PointCloud<double> cloud;
    cloud.points = new L3::Point<double>[10000];
    cloud.num_points = 10000;
    L3::PointCloud<double>::ITERATOR it = cloud.begin();

    boost::mt19937 rng;
    boost::normal_distribution<> normal(0.0, 10.0);

    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, normal );

    while( it != cloud.end() )
        *it++ = L3::Point<double>( var_nor(), var_nor(), var_nor() );


    boost::shared_ptr< L3::HistogramUniformDistance<double> > histogram( new L3::HistogramUniformDistance<double>() );
    histogram->create(0, -50, 50, 0, -50, 50 );

    /*
     *Make histogram
     */
    (*histogram)( &cloud );


    /*
     *Smooth
     */

    L3::Smoother<double,4>* smoother = new L3::BoxSmoother<double,4>();

    smoother->smooth( histogram.get() );

}
