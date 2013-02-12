#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "Datatypes.h"
#include "Dataset.h"
#include "Iterator.h"
#include "Utils.h"
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
    
    //std::string LIDAR_name = dataset.LIDAR_names[0];
    //L3::ConstantTimeIterator* iterator = new L3::ConstantTimeIterator( &dataset, LIDAR_name, 10.0 );
    //L3::Utils::localisePoseChainToMean( dataset.poses );

    glv::Grid grid(glv::Rect(0,0));

    grid.range(1);            // set plot region
    grid.major(1);            // set major tick mark placement
    grid.minor(2);            // number of divisions per major ticks
    grid.equalizeAxes(true);
    grid.stretch(1,.2);

    double d = 800;
    glv::Plot v( glv::Rect( 0, 0, d, d/8), *new glv::PlotFunction1D(glv::Color(0.5,0,0)));
    glv::Plot w( glv::Rect( 0, d/2, d, d/8), *new glv::PlotFunction1D(glv::Color(0.5,0,0)));
   
    top << v << w;

    // Run
    win.setGLV(top);
    glv::Application::run();
}


