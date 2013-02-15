#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"

int main (int argc, char ** argv)
{

    glv::GLV top;
    glv::Window win(1400, 800, "Soaring");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    // Pose sequence
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );
    assert( dataset.validate() && dataset.load() );
    
    std::string LIDAR_name = dataset.LIDAR_names[0];
    L3::ConstantTimeIterator* iterator = new L3::ConstantTimeIterator( &dataset, LIDAR_name, 10.0 );

    L3::Utils::localisePoseChainToMean( dataset.poses );

    glv::Grid grid(glv::Rect(0,0));

    grid.range(1);            // set plot region
    grid.major(1);            // set major tick mark placement
    grid.minor(2);            // number of divisions per major ticks
    grid.equalizeAxes(true);
    grid.stretch(1,.2);

    double d = 800;
    glv::Plot v1__( glv::Rect(    0,0*d/8, d,  d/8), *new glv::PlotFunction1D(glv::Color(0.5,0,0)));

    L3::Visualisers::IteratorRenderer iterator_renderer( iterator  );

    //L3::Visual::PoseChain chain(poses);
    //top << chain << grid;
    //top << chain << v1__ << grid;
    //chain.addHandler(glv::Event::MouseDrag, glv::Behavior::mouseMove); 
    //top << iterator_renderer;

    win.setGLV(top);
    glv::Application::run();
}

