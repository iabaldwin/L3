#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "VisualiserRunner.h"
#include "Components.h"

struct Manipulator : L3::Visualisers::Updateable
{

    
    Manipulator( boost::shared_ptr< L3::PointCloud<double> > cloud, 
                boost::shared_ptr< L3::Estimator::DiscreteEstimator<double> > estimator) 
                    : cloud(cloud),
                        estimator(estimator)
    {
        delta = 1;
    }
    
    int delta;

    boost::shared_ptr< L3::PointCloud<double> > cloud;
    boost::shared_ptr< L3::Estimator::DiscreteEstimator<double> > estimator; 

    void update()
    {
        L3::WriteLock lock( cloud->mutex );
        L3::SE3 pose( delta, 0, 0, 0, 0, 0 );
        L3::translate( cloud.get(), &pose );
        lock.unlock();
    
        estimator->operator()( cloud.get(), L3::SE3::ZERO() );
    }
};


int main (int argc, char ** argv)
{
    /*
     *  Point cloud 1
     */
    boost::shared_ptr< L3::PointCloud<double> > cloud( new L3::PointCloud<double>() );
  
    size_t pts = 1000;

    cloud->points = new L3::Point<double>[pts];
    cloud->num_points = pts;

    // Create a gaussian cloud
    L3::gaussianCloud( cloud.get() );

    std::pair<double,double> ll = L3::min( cloud.get() );
    std::pair<double,double> ur = L3::max( cloud.get() );

    boost::shared_ptr< L3::HistogramUniformDistance<double> > histogram( new L3::HistogramUniformDistance<double>() );
    
    double x_centre =  (ll.first + ur.first)/2.0;
    double y_centre =  (ll.second + ur.second)/2.0;
    histogram->create( x_centre, x_centre-50, x_centre + 50, 
                        y_centre, y_centre - 50, y_centre + 50 );

    histogram->operator()( cloud.get() );



    /*
     *  Point cloud 2
     */

    // Copy
    boost::shared_ptr< L3::PointCloud<double> > cloud_copy( new L3::PointCloud<double>() ); 
    L3::copy( cloud.get(), cloud_copy.get() );
    L3::SE3 pose( 25,30,0,0,0,0 );
    L3::translate( cloud_copy.get(), &pose );

    L3::Estimator::CostFunction<double>* kl_cost_function = new L3::Estimator::KLCostFunction<double>();
    boost::shared_ptr< L3::Estimator::DiscreteEstimator<double> > estimator = boost::make_shared<L3::Estimator::DiscreteEstimator<double> >( kl_cost_function, histogram );

    estimator->pose_estimates.reset( new L3::Estimator::GridEstimates(40, 40, 2) );

    Manipulator manipulator( cloud, estimator );


    /*
     *  Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::PointCloud");

    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    boost::shared_ptr< L3::Visualisers::Updater > updater( new L3::Visualisers::Updater() );
    
    
    (*updater) << &manipulator;
 
    L3::Visualisers::Grid                       grid;
    L3::Visualisers::Composite                  composite;
    L3::Visualisers::BasicPanController         controller( composite.position );
    
    L3::Visualisers::HistogramBoundsRenderer    histogram_bounds_renderer(histogram);
    L3::Visualisers::HistogramDensityRenderer   histogram_density_renderer_view( glv::Rect(600,0,400,400), histogram );

    (*updater) << &histogram_density_renderer_view;

    // Point clouds
    L3::Visualisers::PointCloudRendererLeaf cloud_view( cloud );
    L3::Visualisers::PointCloudRendererLeaf cloud_copy_view( cloud_copy );

    L3::Visualisers::CompositeCloudRendererLeaf point_cloud_composite;

    point_cloud_composite << &cloud_view << &cloud_copy_view;

    // Costs
    L3::Visualisers::CostRendererLeaf cost_renderer(*( estimator->pose_estimates ) );

    // Pose estimates
    boost::shared_ptr< L3::Visualisers::PredictorRenderer > predictor_renderer;
    predictor_renderer.reset( new L3::Visualisers::PredictorRenderer( estimator->pose_estimates ) );
    //composite<<( *(dynamic_cast<L3::Visualisers::Leaf*>(predictor_renderer.get() ))); 

    composite.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) ).stretch(1,1);

    // Add drawables and updateables
    top << (composite << grid << histogram_bounds_renderer << point_cloud_composite << cost_renderer ) << updater.get() << histogram_density_renderer_view;

    // Go
    win.setGLV(top);
    glv::Application::run();

}


