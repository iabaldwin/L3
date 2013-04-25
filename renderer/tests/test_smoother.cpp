#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "Components.h"

int main (int argc, char ** argv)
{
    L3::PointCloud<double> cloud;
    cloud.points = new L3::Point<double>[10000];
    cloud.num_points = 10000;

    L3::gaussianCloud( &cloud );

    boost::shared_ptr< L3::HistogramUniformDistance<double> > histogram_1( new L3::HistogramUniformDistance<double>() );
    histogram_1->create(0, -50, 50, 0, -50, 50 );
    (*histogram_1)( &cloud );

    boost::shared_ptr< L3::HistogramUniformDistance<double> > histogram_2( new L3::HistogramUniformDistance<double>() );
    histogram_2->create(0, -50, 50, 0, -50, 50 );
    (*histogram_2)( &cloud );

    //L3::BoxSmoother< double, 15> smoother;
    //L3::BoxSmoother< double, 5> smoother;
    L3::BoxSmoother< double, 3> smoother;
    smoother.smooth( histogram_2.get() );

    //std::cout << *histogram_2 << std::endl;

    /*
     *Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::PointCloud");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    // Renderer
    L3::Visualisers::Grid                       grid;
    L3::Visualisers::Composite                  composite;
    L3::Visualisers::BasicPanController         controller;
    
    L3::Visualisers::HistogramVertexRenderer    histogram_renderer(histogram_1);
    L3::Visualisers::HistogramBoundsRenderer    histogram_bounds_renderer(histogram_1);
    L3::Visualisers::HistogramDensityRenderer     histogram_pixel_renderer_1( glv::Rect(400,300), histogram_1 );
    L3::Visualisers::HistogramDensityRenderer     histogram_pixel_renderer_2( glv::Rect(410,0,400,300), histogram_2 );

    composite.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) );

    top << (composite << grid << histogram_renderer << histogram_bounds_renderer ) << histogram_pixel_renderer_1 << histogram_pixel_renderer_2 ;

    // Go
    win.setGLV(top);
    glv::Application::run();

}


