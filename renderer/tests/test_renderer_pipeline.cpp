#include <iostream>
#include <fstream>

#include <glv.h>
#include <glv_binding.h>
#include <glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "ViewController.h"

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
    L3::Visualisers::Composite          composite;
    L3::Visualisers::BasicPanController controller( composite.position );

    top << (composite << grid );

    // Go
    win.setGLV(top);
    glv::Application::run();

}
