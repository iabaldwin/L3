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
    //L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" );
    if( !( dataset.validate() && dataset.load() ) )
        throw std::exception();
    
    // Constant time iterator over poses
    L3::ConstantTimeIterator< L3::SE3 >  pose_iterator( dataset.pose_reader, 10.0 );
    // Constant time iterator over LIDAR
    L3::ConstantTimeIterator< L3::LMS151 > LIDAR_iterator( dataset.LIDAR_readers.front(), 10.0 );
    
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
    L3::Visualisers::Composite          composite;
    L3::Visualisers::BasicPanController controller;
    L3::Visualisers::Grid               grid;
    L3::Visualisers::SwatheRenderer     swathe_renderer( &swathe_builder ); 

    composite.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) );
    composite.current_time = time;
    composite.sf = 2.0;

    top << (composite << swathe_renderer << grid );

    win.setGLV(top);
    win.fit(); 
    glv::Application::run();
}


