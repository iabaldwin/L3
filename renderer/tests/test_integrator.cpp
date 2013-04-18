#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "VisualiserRunner.h"

int main (int argc, char ** argv)
{
    /*
     *L3
     */

    L3::Simulator::LHLVGenerator    generator;
    L3::ChainBuilder                pose_windower( &generator );


    /*
     *Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
    
    // Point cloud renderer
    L3::Visualisers::Composite              composite;
    L3::Visualisers::BasicPanController     controller;
    L3::Visualisers::Grid                   grid;
    //L3::Visualisers::SwatheRenderer         swathe_renderer( &swathe_builder ); 
    L3::Visualisers::PoseWindowerRenderer   pose_renderer( &pose_windower ); 

    composite.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) );
    
    L3::Visualisers::VisualiserRunner runner( 0.0 );
    //runner << &swathe_builder << &pose_windower << &generator;
    runner << &pose_windower << &generator;

    top << (composite << pose_renderer << grid  << runner);
    //top << (composite << grid << runner);

    win.setGLV(top);
   
    try
    {
        glv::Application::run();
    }
    catch( L3::end_of_stream& e )
    {
        std::cout << "Done" << std::endl;
    }
}

