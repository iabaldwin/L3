#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"

int main (int argc, char ** argv)
{

    glv::GLV top;
    glv::Window win(1400, 800, "Viewer::Iterator");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    // Pose sequence
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );
    if ( !( dataset.validate() && dataset.load() ) )
        throw std::exception();
    
    L3::ConstantTimeIterator< L3::SE3 > iterator( dataset.pose_reader );

    glv::Grid grid(glv::Rect(0,0));

    grid.range(1);            // set plot region
    grid.major(1);            // set major tick mark placement
    grid.minor(2);            // number of divisions per major ticks
    grid.equalizeAxes(true);
    grid.stretch(1,.2);

    double d = 800;
    glv::Plot v1__( glv::Rect(    0,0*d/8, d,  d/8), *new glv::PlotFunction1D(glv::Color(0.5,0,0)));

    L3::Visualisers::IteratorRenderer<L3::SE3> iterator_renderer( &iterator  );
    L3::Visualisers::Composite composite;
    composite.sf = 10.0;
 
    composite << iterator_renderer;
    composite.current_time = dataset.start_time; 

    //TODO:
    //Not working because??
    top << grid << composite;
    //top << composite << grid;

    win.setGLV(top);
    glv::Application::run();

}


