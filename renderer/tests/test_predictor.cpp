#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "VisualiserRunner.h"

struct Visualiser : L3::Visualisers::Updateable, L3::Visualisers::Leaf
{
    Visualiser( boost::shared_ptr< L3::PointCloud<double> > cloud, 
                boost::shared_ptr< L3::Estimator::DiscreteEstimator<double> > estimator )
                    : cloud(cloud),
                        estimator(estimator)
    {
        cpy_a.reset( new L3::PointCloud<double>() );
   
        L3::WriteLock lock( cloud->mutex );
        L3::copy( cloud.get(), cpy_a.get() );
        lock.unlock();
   
        num_clouds = estimator->pose_estimates->estimates.size();
   
        counter = 0;
        
        cpy_b.reset( new  L3::PointCloud<double>() );
    }
    
       
    int counter;
    int num_clouds;
    boost::shared_ptr< L3::PointCloud<double> > cpy_a;
    boost::shared_ptr< L3::PointCloud<double> > cpy_b;
    boost::shared_ptr< L3::PointCloud<double> > cloud;
    boost::shared_ptr< L3::Estimator::DiscreteEstimator<double> > estimator;


    void update()
    {
        L3::SE3 pose = estimator->pose_estimates->estimates[counter++];

        if (counter == num_clouds)
            counter = 0;
        
        cpy_b.reset( new L3::PointCloud<double>() );
      
        L3::copy( cpy_a.get(), cpy_b.get() );

        L3::WriteLock lock( cpy_b->mutex );
        L3::transform(  cpy_b.get(), &pose );
        lock.unlock();
    
        //estimator->operator()( cloud_copy.get(), L3::SE3::ZERO() );
    }

    void onDraw3D( glv::GLV& g )
    {

        for ( std::vector< L3::SE3 >::iterator it = estimator->pose_estimates->estimates.begin();
                it != estimator->pose_estimates->estimates.end();
                it++ )
        {
            //L3::Visualisers::CoordinateSystem( *it).onDraw3D( g) ;
        }

        L3::ReadLock lock( cpy_b->mutex );
        L3::Visualisers::PointCloudRendererLeaf( cpy_b ).onDraw3D( g );
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
    L3::gaussianCloud( cloud.get() );
    
    // Make a copy
    boost::shared_ptr< L3::PointCloud<double> > cloud_copy( new L3::PointCloud<double>() );
    L3::copy( cloud.get(), cloud_copy.get() );

    L3::SE3 delta( -25, -25, 0, 0, 0, 0 );
    L3::transform( cloud.get(), &delta );

    boost::shared_ptr< L3::HistogramUniformDistance<double> > histogram( new L3::HistogramUniformDistance<double>() );
    
    histogram->create( 0, -50, 50, 
                        0, -50, 50 );

    histogram->operator()( cloud.get() );

    L3::Estimator::CostFunction<double>* kl_cost_function = new L3::Estimator::KLCostFunction<double>();
    boost::shared_ptr< L3::Estimator::DiscreteEstimator<double> > estimator = boost::make_shared<L3::Estimator::DiscreteEstimator<double> >( kl_cost_function, histogram );

    estimator->pose_estimates.reset( new L3::Estimator::GridEstimates(40, 40, 2) );

    estimator->operator()( cloud_copy.get(), L3::SE3::ZERO() );
                
    Visualiser visualiser( cloud_copy, estimator );

    /*
     *  Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::PointCloud");

    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    boost::shared_ptr< L3::Visualisers::Updater > updater( new L3::Visualisers::Updater() );
  
    (*updater) << &visualiser;

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

    // Pose estimates
    boost::shared_ptr< L3::Visualisers::PredictorRenderer > predictor_renderer;
    predictor_renderer.reset( new L3::Visualisers::PredictorRenderer( estimator->pose_estimates ) );
    //composite<<( *(dynamic_cast<L3::Visualisers::Leaf*>(predictor_renderer.get() ))); 

    composite.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) ).stretch(1,1);

    // Add drawables and updateables
    top << (composite << grid << histogram_bounds_renderer << point_cloud_composite << cost_renderer << visualiser ) << updater.get() << histogram_density_renderer_view;

    // Go
    win.setGLV(top);
    glv::Application::run();

}


