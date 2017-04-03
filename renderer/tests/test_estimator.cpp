#include <iostream>
#include <fstream>

#include <glv.h>
#include <glv_binding.h>
#include <glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "VisualiserRunner.h"

struct Visualiser : L3::Visualisers::Updateable, L3::Visualisers::Leaf
{
    Visualiser( boost::shared_ptr< L3::PointCloud<double> > experience_cloud, 
                boost::shared_ptr< L3::Estimator::DiscreteEstimator<double> > estimator ,
                boost::shared_ptr< L3::HistogramUniformDistance<double> > histogram )
                    : experience_cloud(experience_cloud),
                        experience_histogram( histogram ),
                        estimator(estimator), 
                        num_estimates( estimator->pose_estimates->estimates.size() ),
                        counter(0)
    {
        experience_cloud_copy.reset( new L3::PointCloud<double>() );
    
    }
    
       
    int counter;
    int num_estimates;
    boost::shared_ptr< L3::PointCloud<double> > experience_cloud;
    boost::shared_ptr< L3::PointCloud<double> > experience_cloud_copy;
    boost::shared_ptr< L3::HistogramUniformDistance<double> > experience_histogram ;
    boost::shared_ptr< L3::Estimator::DiscreteEstimator<double> > estimator;


    void update()
    {
        L3::SE3 pose = estimator->pose_estimates->estimates[counter++];

        if (counter == num_estimates)
            counter = 0;
        
        experience_cloud_copy.reset( new L3::PointCloud<double>() );
      
        // Move the experience cloud
        L3::ReadLock read_lock( experience_cloud->mutex );
        L3::ReadLock write_lock( experience_cloud_copy->mutex );
        
        L3::copy( experience_cloud.get(), experience_cloud_copy.get() );
        L3::transform(  experience_cloud_copy.get(), &pose );
        
        read_lock.unlock();
        write_lock.unlock();

        // Re-histogram it
        experience_histogram->clear();
        experience_histogram->operator()( experience_cloud_copy.get() );
        
        // Re-estimate 
        estimator->operator()( experience_cloud.get(), L3::SE3::ZERO() );
    
    }

    void onDraw3D( glv::GLV& g )
    {
        L3::ReadLock lock( experience_cloud_copy->mutex );
        L3::Visualisers::PointCloudRendererLeaf( experience_cloud_copy ).onDraw3D( g );
        lock.unlock();
    }
};


int main (int argc, char ** argv)
{
    /*
     *  Point cloud 1
     */
    boost::shared_ptr< L3::PointCloud<double> > cloud( new L3::PointCloud<double>() );
  
    size_t pts = 10*1000;

    cloud->points = new L3::Point<double>[pts];
    cloud->num_points = pts;

    // Create a gaussian cloud
    //L3::gaussianCloud( cloud.get() );
    L3::gaussianCloud( cloud.get(), 5, 15 );
    
    // Make a copy
    boost::shared_ptr< L3::PointCloud<double> > cloud_copy( new L3::PointCloud<double>() );
    L3::copy( cloud.get(), cloud_copy.get() );

    boost::shared_ptr< L3::HistogramUniformDistance<double> > histogram( new L3::HistogramUniformDistance<double>(2.0) );
    
    histogram->create( 0, -50, 50, 
                        0, -50, 50 );

    histogram->operator()( cloud.get() );

    L3::Estimator::CostFunction<double>* kl_cost_function = new L3::Estimator::KLCostFunction<double>();
    boost::shared_ptr< L3::Estimator::GridEstimates >  xy_grid( new L3::Estimator::GridEstimates( 40, 40, 2) );
    boost::shared_ptr< L3::Estimator::DiscreteEstimator<double> > estimator = boost::make_shared<L3::Estimator::DiscreteEstimator<double> >( kl_cost_function, histogram, xy_grid );
    estimator->operator()( cloud_copy.get(), L3::SE3::ZERO() );
               
    /*
     *  Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::PointCloud");

    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    boost::shared_ptr< L3::Visualisers::Updater > updater( new L3::Visualisers::Updater() );
  
    L3::Visualisers::Grid                       grid;
    L3::Visualisers::Composite                  composite;
    L3::Visualisers::BasicPanController         controller( composite.position );
    
    L3::Visualisers::HistogramBoundsRenderer    histogram_bounds_renderer(histogram);
    
    L3::Visualisers::HistogramDensityRenderer   histogram_density_renderer_view( glv::Rect(600,0,400,400), histogram );
    (*updater) << &histogram_density_renderer_view;

    // Point clouds
    L3::Visualisers::PointCloudRendererLeaf cloud_view( cloud );

    L3::Visualisers::CompositeCloudRendererLeaf point_cloud_composite;

    point_cloud_composite << &cloud_view;

    // Costs
    L3::Visualisers::CostRendererLeaf cost_renderer(*( estimator->pose_estimates ) );

    Visualiser visualiser( cloud_copy, estimator, histogram  );
    (*updater) << &visualiser;

    composite.stretch(1,1);

    // Add drawables and updateables
    top << (composite << grid << histogram_bounds_renderer << point_cloud_composite << cost_renderer << visualiser ) << updater.get() << histogram_density_renderer_view;
    //top << (composite << grid << point_cloud_composite << cost_renderer << visualiser ) << updater.get();

    // Go
    win.setGLV(top);
    glv::Application::run();

}


