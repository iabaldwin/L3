#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "Datatypes.h"
#include "Utils.h"
#include "Reader.h"
#include "Writer.h"
#include "Visualisers.h"

int main (int argc, char ** argv)
{

    glv::GLV top;
    glv::Window win(1400, 800, "Soaring");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    // Pose sequence
    std::vector<L3::Pose*> poses;
    std::auto_ptr<L3::IO::PoseReader> reader( new L3::IO::PoseReader() );
    
    reader->open( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/L3/OxTS.ins" );
    reader->read();

    if ( !reader->extract( poses ) )
        throw std::exception();

    //L3::Utils::localisePoseChainToOrigin( poses );
    L3::Utils::localisePoseChainToMean( poses );

    std::vector<L3::Pose*>::iterator it = poses.begin(); 
    while( it != poses.end() )
    {
        std::cout << *(*it) << std::endl;
        it++;
    }

    glv::Grid grid(glv::Rect(0,0));

    grid.range(1);            // set plot region
    grid.major(1);            // set major tick mark placement
    grid.minor(2);            // number of divisions per major ticks
    grid.equalizeAxes(true);
    grid.stretch(1,.2);

    double d = 800;
    glv::Plot v1__( glv::Rect(    0,0*d/8, d,  d/8), *new glv::PlotFunction1D(glv::Color(0.5,0,0)));

    L3::Visualisers::PoseChainRenderer chain(poses);
    //top << chain << grid;
    //top << chain << v1__ << grid;
    //chain.addHandler(glv::Event::MouseDrag, glv::Behavior::mouseMove); 
        
    win.setGLV(top);
    glv::Application::run();
}


