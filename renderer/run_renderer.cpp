#include "Components.h"
#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>


int main (int argc, char ** argv){

    glv::GLV top;
    glv::Window win(1200, 600, "Soaring");
	
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    //Scene scene;
    //top << scene;

    L3::PoseChain chain;
    top << chain;

    win.setGLV(top);
    glv::Application::run();
}


