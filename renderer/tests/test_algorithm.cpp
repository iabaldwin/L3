#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "VisualiserRunner.h"

struct AlgorithmVisualiser : L3::Visualisers::Leaf
{

    AlgorithmVisualiser( boost::shared_ptr< L3::Estimator::IterativeDescent<double> > estimator ) :estimator(estimator)
    {

    }
    
    boost::shared_ptr< L3::Estimator::IterativeDescent<double> > estimator;

    void onDraw3D( glv::GLV& g )
    {
        std::deque< boost::shared_ptr< L3::Estimator::DiscreteEstimator<double> > >::iterator it = estimator->discrete_estimators.begin() ;

        float height = 10.0;

        while( it != estimator->discrete_estimators.end() )
        {
            glv::Point3 vertices[ (*it)->pose_estimates->estimates.size() ];
            glv::Color  colors[ (*it)->pose_estimates->estimates.size() ];

            int counter = 0;
            for(  std::vector< L3::SE3 > ::iterator pose_it = (*it)->pose_estimates->estimates.begin();
                    pose_it != (*it)->pose_estimates->estimates.end();
                    pose_it++ )
            {
                vertices[counter++]( pose_it->X(), pose_it->Y(), height+ (*it)->pose_estimates->costs[ std::distance( (*it)->pose_estimates->estimates.begin(), pose_it )] );
            }
        
            glv::draw::paint( glv::draw::Points, vertices, colors, counter );
            height += 10;

            it++;
        }
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

    std::cout << "Creating... ";
    boost::shared_ptr< L3::Estimator::IterativeDescent<double> > estimator( new L3::Estimator::IterativeDescent<double> ( kl_cost_function, pyramid ) );
    std::cout << "Done" << std::endl;

    std::cout << "Estimating ... "; std::cout.flush();
    estimator->operator()( cloud_copy.get(), L3::SE3::ZERO() );
    std::cout << "Done" << std::endl;

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

    AlgorithmVisualiser                     algo_visualiser( estimator );

    //L3::Visualisers::HistogramBoundsRenderer    histogram_bounds_renderer(histogram);
    
    //L3::Visualisers::HistogramDensityRenderer   histogram_density_renderer_view( glv::Rect(600,0,400,400), histogram );
    //(*updater) << &histogram_density_renderer_view;

    // Point clouds
    L3::Visualisers::PointCloudRendererLeaf cloud_view( cloud );

    L3::Visualisers::CompositeCloudRendererLeaf point_cloud_composite;

    point_cloud_composite << &cloud_view;

    // Costs
    //L3::Visualisers::CostRendererLeaf cost_renderer(*( estimator->pose_estimates ) );

    //Visualiser visualiser( cloud_copy, estimator, histogram  );
    //(*updater) << &visualiser;

    composite.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) ).stretch(1,1);

    // Add drawables and updateables
    //top << (composite << grid << histogram_bounds_renderer << point_cloud_composite << cost_renderer << visualiser ) << updater.get() << histogram_density_renderer_view;
    //top << (composite << grid << point_cloud_composite << cost_renderer << visualiser ) << updater.get();
    //top << (composite << grid << point_cloud_composite << visualiser ) << updater.get();
    top << (composite << grid << point_cloud_composite << algo_visualiser ) << updater.get();

    // Go
    win.setGLV(top);
    glv::Application::run();

}


