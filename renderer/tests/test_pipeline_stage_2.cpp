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

    experience_renderer.addPoseProvider( &pose_windower );

    composite.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) );
    composite.current_time = time;
    composite.sf = 2.0;

    double d = 800;
    glv::Plot velocity( glv::Rect( 0,0*d/8, d, d/8), *new glv::PlotFunction1D(glv::Color(0.5,0,0)));

    top << (composite << swathe_renderer << grid << experience_renderer ) << velocity;

    win.setGLV(top);
    win.fit(); 
    glv::Application::run();
}


