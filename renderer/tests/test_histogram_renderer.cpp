#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>


#include "L3.h"
#include "Visualisers.h"
#include "VisualiserRunner.h"
#include "Components.h"

int main (int argc, char ** argv)
{
    L3::PointCloud<double> cloud;
    cloud.points = new L3::Point<double>[10000];
    cloud.num_points = 10000;

    L3::gaussianCloud( &cloud );

    boost::shared_ptr< L3::HistogramUniformDistance<double> > histogram( new L3::HistogramUniformDistance<double>() );
    histogram->create(0, -50, 50, 0, -50, 50 );

    (*histogram)( &cloud );

    /*
     *Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::PointCloud");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    // Renderer
    boost::shared_ptr< L3::Visualisers::Updater > updater( new L3::Visualisers::Updater() );
 
    L3::Visualisers::Grid                       grid;
    L3::Visualisers::Composite                  composite;
    L3::Visualisers::BasicPanController         controller;
    
    L3::Visualisers::HistogramVertexRenderer    histogram_renderer(histogram);
    L3::Visualisers::HistogramBoundsRenderer    histogram_bounds_renderer(histogram);
    L3::Visualisers::HistogramDensityRenderer   histogram_density_renderer( glv::Rect(500,300), histogram );
    L3::Visualisers::HistogramVoxelRenderer     histogram_voxel_renderer( histogram );

    composite.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) ).stretch(1,1);

    updater->operator<< (dynamic_cast<L3::Visualisers::Updateable*>(&histogram_density_renderer));
    updater->operator<< (dynamic_cast<L3::Visualisers::Updateable*>(&histogram_voxel_renderer));

    // Add drawables and updateables
    top << (composite << grid << histogram_renderer <<  histogram_bounds_renderer ) << histogram_density_renderer << &histogram_voxel_renderer <<  updater.get();

    // Go
    win.setGLV(top);
    glv::Application::run();

}


