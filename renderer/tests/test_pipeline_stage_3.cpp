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
     *L3
     */
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" );
    if( !( dataset.validate() && dataset.load() ) )
        throw std::exception();

    // Configuration
    L3::Configuration::Mission mission( dataset );

    L3::Dataset experience_dataset( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" );
    L3::ExperienceLoader experience_loader( experience_dataset );
    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;

    // Constant time iterator over poses
    L3::ConstantTimeIterator< L3::SE3 >  pose_iterator( dataset.pose_reader );
    
    // Constant time iterator over LIDAR
    L3::ConstantTimeIterator< L3::LMS151 > LIDAR_iterator( dataset.LIDAR_readers[ mission.declined ] );
  
    double time = dataset.start_time;

    L3::ConstantTimePoseWindower pose_windower( &pose_iterator );
    
    L3::SwatheBuilder swathe_builder( &pose_windower, &LIDAR_iterator );

    L3::Estimator::CostFunction<double>* kl_cost_function = new L3::Estimator::KLCostFunction<double>();
    L3::Estimator::DiscreteEstimator<double> estimator( kl_cost_function );

    /*
     *Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::PointCloud");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
  
    L3::Visualisers::Composite                  composite;
    L3::Visualisers::BasicPanController         controller;
    L3::Visualisers::Grid                       grid;
    L3::Visualisers::SwatheRenderer             swathe_renderer( &swathe_builder ); 
    L3::Visualisers::ExperienceRenderer         experience_renderer( experience );
    //L3::Visualisers::HistogramPixelRenderer histogram_renderer( glv::Rect(500,300) );
    //L3::Visualisers::HistogramBoundsRenderer    histogram_renderer;
    L3::Visualisers::HistogramVertexRenderer    histogram_renderer;

    histogram_renderer( estimator.experience_histogram );


    L3::SE3 projection(0,0,0,.1,.2,.3);
    L3::PointCloud<double>* point_cloud = new L3::PointCloud<double>();
    std::auto_ptr< L3::Projector<double> > projector( new L3::Projector<double>( &projection, point_cloud) );

    // Link the experience to the current pose generator
    experience_renderer.addPoseProvider( &pose_windower );

    composite.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) );
    composite.current_time = time;
    composite.sf = 2.0;

    // Add watchers
    composite << swathe_renderer << grid << experience_renderer << histogram_renderer;

    // Create runner
    L3::EstimatorRunner runner;
    
    runner.setExperience( &*experience )
          .setPoseProvider( &pose_windower )
          .setProjector( &*projector )
          .setEstimator( &estimator  )
          .setSwatheBuilder( &swathe_builder );

    L3::Visualisers::Runner runner_visualiser;
    runner_visualiser << &runner;

    //top << (composite << runner_visualiser ) << histogram_renderer;
    top << (composite << runner_visualiser );

    win.setGLV(top);
    win.fit(); 
    glv::Application::run();

}

