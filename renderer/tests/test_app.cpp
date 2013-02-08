#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "Datatypes.h"
#include "Utils.h"
#include "Reader.h"
#include "Writer.h"
#include "Components.h"

int main (int argc, char ** argv)
{

    glv::GLV top;
    glv::Window win(1200, 600, "Soaring");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    // Pose sequence
    std::vector<L3::Pose*> poses;
    std::auto_ptr<L3::IO::PoseReader> reader( new L3::IO::PoseReader() );
    
    reader->open( "/Users/ian/code/python/tools/test/OxTS.ins" );
    reader->read();

    if ( !reader->extract( poses ) )
        throw std::exception();

    L3::UTILS::localisePoseChainToOrigin( poses );

    glv::Grid grid(glv::Rect(0,0));

    grid.range(1);            // set plot region
    grid.major(1);            // set major tick mark placement
    grid.minor(2);            // number of divisions per major ticks
    grid.equalizeAxes(true);
    grid.stretch(1,.2);


    L3::Visual::PoseChain chain(poses);
    //top << chain << grid;
    top << chain;

    chain.addHandler(glv::Event::MouseDrag, glv::Behavior::mouseMove); 
        
    win.setGLV(top);
    glv::Application::run();
}


