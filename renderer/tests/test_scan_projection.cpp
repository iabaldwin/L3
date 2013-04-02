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

    // Constant time iterator over poses
    L3::ConstantTimeIterator< L3::SE3 >  pose_iterator( dataset.pose_reader );
    
    // Constant time iterator over LIDAR - declined
    L3::ConstantTimeIterator< L3::LMS151 > declined_lidar( dataset.LIDAR_readers[ mission.declined ] );
    
    // Constant time iterator over LIDAR - horizontal
    L3::ConstantTimeIterator< L3::LMS151 > horizontal_lidar( dataset.LIDAR_readers[ mission.horizontal ] );

    double time = dataset.start_time;

    // Windowed pose producer
    L3::ConstantTimePoseWindower pose_windower( &pose_iterator );
    
    // Swathe builder
    L3::SwatheBuilder swathe_builder( &pose_windower, &declined_lidar );
    
    // Build runner
    L3::TemporalRunner t;

    /*
     *Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::ScanRenderer");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    // Point cloud renderer
    L3::Visualisers::Composite      composite;
    L3::Visualisers::Controller*    controller = new L3::Visualisers::BasicPanController();
    L3::Visualisers::Grid           grid;
    L3::Visualisers::ScanRenderer   scan_renderer( &swathe_builder ); 

    composite.addController( controller );
    composite.current_time = time;
    composite.sf = 2.0;

    // Compose 
    top << (composite << scan_renderer << grid) ;

    win.setGLV(top);
    win.fit(); 
 

    try
    {
        glv::Application::run();
    }
    catch( ... )
    {
    }
}