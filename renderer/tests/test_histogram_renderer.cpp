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
    L3::PointCloud<double>::ITERATOR it = cloud.begin();
    
    while( it != cloud.end() )
    {
        *it++ = L3::Point<double>( random()%100, random()%100, random()%100 );
    }

    boost::shared_ptr< L3::Histogram<double> > histogram( new L3::Histogram<double>( L3::Histogram<double>::UniformDistance(0, -50, 50, 0, -50, 50, 1 ) ) );

    (*histogram)( &cloud );

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
    L3::Visualisers::HistogramVertexRenderer    histogram_renderer;
 
    L3::Visualisers::HistogramPixelRenderer     histogram_pixel_renderer( glv::Rect(500,300), );

    histogram_renderer( &histogram );
    histogram_pixel_renderer( &histogram );

    composite.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) );

    // Add drawables and updateables
    top << (composite << grid << histogram_renderer << histogram_pixel_renderer  );

    // Add drawables
    top << histogram_pixel_renderer;

    // Go
    win.setGLV(top);
    glv::Application::run();

}


