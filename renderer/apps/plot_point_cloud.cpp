#include <iostream>
#include <vector>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>


#include "L3.h"
#include "Visualisers.h"
#include "Controller.h"


template <typename T>
L3::Point<T> randomate()
{
    return L3::Point<T>( random() % 1000, random() % 1000, random() % 1000  );
}

int main()
{
    /*
     *Build cloud
     */
    L3::PointCloud<double>*  cloud = new L3::PointCloud<double>();

    std::vector< L3::Point<double> > randoms(2*100000);

    std::generate( randoms.begin(), randoms.end(), randomate<double> );

    cloud->points = &randoms[0];
    cloud->num_points = randoms.size();

    /*
     *Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::PointCloud");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
    
    glv::Grid grid(glv::Rect(0,0));
    grid.range(1);            // set plot region
    grid.major(1);            // set major tick mark placement
    grid.minor(2);            // number of divisions per major ticks
    grid.equalizeAxes(true);
    grid.stretch(1,.2);

    double d = 800;
    glv::Plot v1__( glv::Rect( 0,0*d/8, d, d/8), *new glv::PlotFunction1D(glv::Color(0.5,0,0)));

    // Point cloud renderer
    L3::Visualisers::CloudRenderer<double> cloud_renderer( cloud );
    L3::Visualisers::Composite composite_view;
  
    composite_view << cloud_renderer;
    top << composite_view;

    win.setGLV(top);
    glv::Application::run();

}
