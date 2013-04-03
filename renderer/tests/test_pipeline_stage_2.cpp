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

    L3::Configuration::Mission mission( dataset );

    L3::ExperienceLoader experience_loader( dataset );
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
    
    // Point cloud renderer
    L3::Visualisers::Composite              composite;
    L3::Visualisers::BasicPanController     controller;
    L3::Visualisers::Grid                   grid;
    L3::Visualisers::SwatheRenderer         swathe_renderer( &swathe_builder ); 
    L3::Visualisers::ExperienceRenderer     experience_renderer( experience );


    // Link the experience to the current pose generator
    experience_renderer.addPoseProvider( &pose_windower );

    composite.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) );
    composite.current_time = time;
    composite.sf = 2.0;

    // Add watchers
    composite << swathe_renderer << grid << experience_renderer;

    // Add runner
    L3::Visualisers::Runner runner;    
    runner << &swathe_builder << &pose_windower;

    top << (composite << runner );

    win.setGLV(top);
    win.fit(); 
    glv::Application::run();

}

