#include <iostream>
#include <fstream>

#include "L3.h"

int main (int argc, char ** argv)
{
    /*
     *  Point cloud 1
     */
    boost::shared_ptr< L3::PointCloud<double> > cloud( new L3::PointCloud<double>() );
  
    size_t pts = 10*1000;

    cloud->points = new L3::Point<double>[pts];
    cloud->num_points = pts;

    /*
     *Create a gaussian cloud
     */
    L3::gaussianCloud( cloud.get(), 5, 15 );
    
    /*
     *Make a copy
     */
    boost::shared_ptr< L3::PointCloud<double> > cloud_copy( new L3::PointCloud<double>() );
    L3::copy( cloud.get(), cloud_copy.get() );

    std::vector<double> densities;
    densities.push_back(1);
    densities.push_back(5);
    densities.push_back(10);

    boost::shared_ptr< L3::HistogramPyramid<double> > pyramid( new L3::HistogramPyramid<double>( densities ) );

    for( L3::HistogramPyramid<double>::PYRAMID_ITERATOR it = pyramid->begin();
            it != pyramid->end();
            it++ )
    {
        boost::shared_ptr<L3::HistogramUniformDistance<double> > current_histogram = 
            boost::dynamic_pointer_cast<L3::HistogramUniformDistance<double> >(*it);
        
        current_histogram->create( 0, -50, 50, 
                        0, -50, 50 );
   
        current_histogram->operator()( cloud_copy.get() );
    }


    L3::Estimator::CostFunction<double>* kl_cost_function = new L3::Estimator::KLCostFunction<double>();
    boost::shared_ptr< L3::Estimator::IterativeDescent<double> > estimator( new L3::Estimator::IterativeDescent<double> ( kl_cost_function, pyramid ) );
    estimator->operator()( cloud_copy.get(), L3::SE3::ZERO() );


    L3::Estimator::Algorithm<double>* algorithm = new L3::Estimator::Minimisation<double>( kl_cost_function, pyramid );

}
