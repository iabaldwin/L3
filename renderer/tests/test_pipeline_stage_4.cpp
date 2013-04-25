#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"

int main (int argc, char ** argv)
{
    /*
     *  L3
     */
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" );
    if( !( dataset.validate() && dataset.load() ) )
        throw std::exception();

    // Configuration
    L3::Configuration::Mission mission( dataset );

    // Experience
    L3::Dataset experience_dataset( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" );
    L3::ExperienceLoader experience_loader( experience_dataset );
    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;

    // Constant time iterator over poses
    L3::ConstantTimeIterator< L3::SE3 >  pose_iterator( dataset.pose_reader );
    
    // Constant time iterator over LIDAR
    L3::ConstantTimeIterator< L3::LMS151 > LIDAR_iterator( dataset.LIDAR_readers[ mission.declined ] );
  
    // Pose Windower
    L3::ConstantTimeWindower<L3::SE3> pose_windower( &pose_iterator );
    
    // Swathe builder
    L3::SwatheBuilder swathe_builder( &pose_windower, &LIDAR_iterator );

    // Projection
    L3::SE3 projection(0,0,0,.1,.2,.3);
    L3::PointCloud<double>* point_cloud = new L3::PointCloud<double>();
    std::auto_ptr< L3::Projector<double> > projector( new L3::Projector<double>( &projection, point_cloud) );

    // Estimator
    L3::Estimator::CostFunction<double>* kl_cost_function = new L3::Estimator::KLCostFunction<double>();
    L3::Estimator::DiscreteEstimator<double> estimator( kl_cost_function, experience->experience_histogram );

    // Create threaded runner
    L3::EstimatorRunner runner;
    
    runner.setExperience( &*experience )
          .setPoseProvider( &pose_windower )
          .setProjector( &*projector )
          .setEstimator( &estimator  )
          .setSwatheBuilder( &swathe_builder )
          .start( dataset.start_time );
    
    /*
     *Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::Stage4");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
  
    L3::Visualisers::Composite                  composite;      // 3D composer
    L3::Visualisers::BasicPanController         controller;     // Control type
    L3::Visualisers::Grid                       grid;           // Grid spacing
    L3::Visualisers::ExperienceRenderer         experience_renderer( experience );
    L3::Visualisers::HistogramDensityRenderer     histogram_pixel_renderer_experience( glv::Rect(50, 200, 500, 300 ), experience->experience_histogram );
    L3::Visualisers::HistogramDensityRenderer     histogram_pixel_renderer_swathe( glv::Rect(50, 400, 500, 300 ), estimator.swathe_histogram );
    L3::Visualisers::HistogramBoundsRenderer    histogram_bounds_renderer( experience->experience_histogram );
    L3::Visualisers::PointCloudBoundsRenderer   point_cloud_bounds_renderer( point_cloud );
    L3::Visualisers::PointCloudRenderer         runtime_cloud_renderer( point_cloud );
    L3::Visualisers::PoseEstimatesRenderer      pose_estimates_renderer( estimator.pose_estimates );
    
    composite.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) ).stretch(1,1);

    // Add 3D watchers
    composite << grid 
                << runtime_cloud_renderer 
                << histogram_bounds_renderer 
                << point_cloud_bounds_renderer
                << experience_renderer 
                << pose_estimates_renderer 
                ;

   
    // Add top level components
    top << (composite ) 
        << histogram_pixel_renderer_experience 
        << histogram_pixel_renderer_swathe;

    win.setGLV(top);
   
    try
    {
        glv::Application::run();
    }
    catch( ... )
    {
        std::cout << "Done" << std::endl;
    }
}

