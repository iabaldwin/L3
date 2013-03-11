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
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-08-09-36-42-WOODSTOCK-SLOW/" );
    if( !( dataset.validate() && dataset.load() ) )
        throw std::exception();

    L3::Experience experience( dataset );

    /*
     *Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::ExperienceRenderer");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
    
    // Point cloud renderer
    L3::Visualisers::Composite          composite;
    //L3::Visualisers::Controller*        controller = new L3::Visualisers::BasicPanController();
    L3::Visualisers::BasicPanController controller;
    L3::Visualisers::Grid               grid;
    L3::Visualisers::ExperienceRenderer experience_renderer( &experience );

    composite.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) );
    
    top << (composite << grid << experience_renderer) ;

    win.setGLV(top);
    win.fit(); 
    glv::Application::run();

}
