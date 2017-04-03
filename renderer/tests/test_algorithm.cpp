#include <iostream>
#include <fstream>

#include <glv.h>
#include <glv_binding.h>
#include <glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "VisualiserRunner.h"


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

    boost::shared_ptr< L3::HistogramPyramid<double> > pyramid( new L3::HistogramPyramid<double>(densities ) );

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

    /*
     *  Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::PointCloud");

    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    boost::shared_ptr< L3::Visualisers::Updater > updater( new L3::Visualisers::Updater() );
  
    L3::Visualisers::Grid                           grid;
    L3::Visualisers::Composite                      composite;
    L3::Visualisers::BasicPanController             controller( composite.position );
    L3::Visualisers::HistogramPyramidRendererView   pyramid_renderer( pyramid, 3 );

    for ( std::deque< boost::shared_ptr< L3::Visualisers::HistogramDensityRenderer > >::iterator it = pyramid_renderer.renderers.begin();
                    it != pyramid_renderer.renderers.end();
                    it++ )
            updater->operator<<( it->get() );

    // Point clouds
    L3::Visualisers::PointCloudRendererLeaf cloud_view( cloud );

    L3::Visualisers::CompositeCloudRendererLeaf point_cloud_composite;

    point_cloud_composite << &cloud_view;

    // Costs
    composite.stretch(1,1);

    // Add drawables and updateables
    top << (composite << grid << point_cloud_composite ) << updater.get() << pyramid_renderer;

    // Go
    win.setGLV(top);
    glv::Application::run();

}


