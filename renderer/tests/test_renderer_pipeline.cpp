#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "Controller.h"

int main (int argc, char ** argv)
{
    /*
     *Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::PointCloud");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
    
    // Renderer
    L3::Visualisers::Grid               grid;
    L3::Visualisers::Controller         controller;
    L3::Visualisers::Composite          composite_view;

    top << (composite_view << grid );

    //top.addHandler( glv::Event::KeyDown, controller );
    //top.addHandler( glv::Event::KeyRepeat, controller );
   
    // Go
    win.setGLV(top);
    glv::Application::run();

}
