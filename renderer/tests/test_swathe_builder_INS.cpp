#include <iostream>
#include <fstream>

#include <glv.h>
#include <glv_binding.h>
#include <glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "VisualiserRunner.h"

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
    
    // Constant time iterator over LIDAR
    L3::ConstantTimeIterator< L3::LMS151 > LIDAR_iterator( dataset.LIDAR_readers[ mission.declined ] );
    
    double time = dataset.start_time;

    L3::ConstantTimeWindower<L3::SE3> pose_windower( &pose_iterator );
    
    L3::RawSwatheBuilder swathe_builder( &pose_windower, &LIDAR_iterator );

    /*
     *Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::PointCloud");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
    
    // Point cloud renderer
    L3::Visualisers::Composite              composite;
    L3::Visualisers::BasicPanController     controller( composite.position );
    L3::Visualisers::Grid                   grid;
    L3::Visualisers::SwatheRenderer         swathe_renderer( &swathe_builder ); 
    L3::Visualisers::PoseWindowerRenderer   pose_renderer( &pose_windower ); 

    composite.stretch(1,1);

    L3::Visualisers::VisualiserRunner runner( dataset.start_time );
    runner << &swathe_builder << &pose_windower << &LIDAR_iterator << &pose_iterator ;

    // Add watchers
    composite << swathe_renderer << pose_renderer <<  grid << runner;

    top << (composite);

    win.setGLV(top);
    
    try
    {
        glv::Application::run();
    }
    catch( L3::end_of_stream& e )
    {
        std::cout << "Fin" << std::endl;
    }
}


