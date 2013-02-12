#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "Visualisers.h"

int main (int argc, char ** argv){

    glv::GLV top;
    glv::Window win(1200, 600, "Soaring");
	
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    glv::Grid grid(glv::Rect(0,0));

    grid.range(1);            // set plot region
    grid.major(1);            // set major tick mark placement
    grid.minor(2);            // number of divisions per major ticks
    grid.equalizeAxes(true);
    grid.stretch(1,.2);


    top << grid;

    win.setGLV(top);
    glv::Application::run();
}


