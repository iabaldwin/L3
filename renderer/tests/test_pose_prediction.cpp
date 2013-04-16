#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include <boost/date_time/posix_time/posix_time_types.hpp>

#include "L3.h"
#include "Visualisers.h"

struct VisualiserRunner : L3::Visualisers::Leaf, L3::TemporalRunner
{

    VisualiserRunner( double start_time ) : time(start_time)
    {
    }

    double time;

    void onDraw3D( glv::GLV& g )
    {
        this->update( time += .5 );
    }

};


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
    L3::Visualisers::PoseWindowerRenderer   pose_renderer( &pose_windower ); 

    composite.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) );
    composite.current_time = time;
    composite.sf = 2.0;

    // Add watchers
    composite << swathe_renderer << pose_renderer <<  grid;

    // Add runner
    VisualiserRunner runner( dataset.start_time );
    runner << &swathe_builder << &pose_windower;

    top << (composite << runner);

    win.setGLV(top);
    win.fit(); 
    
    try
    {
        glv::Application::run();
    }
    catch( L3::end_of_stream& e )
    {
        std::cout << "Done" << std::endl;
    }
}


