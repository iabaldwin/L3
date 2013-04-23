#include <iostream>

#include "L3.h"

int main (int argc, char ** argv)
{
    L3::PointCloud<double> cloud;
    cloud.points = new L3::Point<double>[10000];
    cloud.num_points = 10000;

    L3::gaussianCloud( &cloud );

    std::ofstream output( "test.dat" );

    std::copy( cloud.begin(),
            cloud.end(),
            std::ostream_iterator<L3::Point<double> >( output, "\n" ) ); 

    output.close();
   

    boost::shared_ptr< L3::HistogramUniformDistance<double> > histogram( new L3::HistogramUniformDistance<double>( 10 ) );
    histogram->create(0, -50, 50, 0, -50, 50 );

    (*histogram)( &cloud );

    std::list< boost::shared_ptr< L3::SmootherInterface > > histogram_pyramid( 3 );

    histogram_pyramid.push_back( boost::make_shared< L3::BoxSmoother<double,15 > >() );
    histogram_pyramid.push_back( boost::make_shared< L3::BoxSmoother<double,7 > >() );
    histogram_pyramid.push_back( boost::make_shared< L3::BoxSmoother<double,3 > >() );

    for( std::list< boost::shared_ptr< L3::SmootherInterface > >::iterator it = histogram_pyramid.begin(); 
            it != histogram_pyramid.end(); 
            it++ )
    {

        boost::shared_ptr< L3::HistogramUniformDistance<double> > histogram_copy( new L3::HistogramUniformDistance<double>() );
        L3::clone( histogram.get(), histogram_copy.get() );

        //dynamic_cast<L3::BoxSmoother<double,6> >(it->get())->smooth( histogram_copy.get() );

        //std::ofstream histogram_out( "histogram.original" );
        //histogram_out << *histogram;
        //histogram_out.close();

        //histogram_out.open( "histogram.smoothed" );
        //histogram_out << *histogram_copy;
        //histogram_out.close();

    }

}


