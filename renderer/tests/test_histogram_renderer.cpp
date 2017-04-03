#include <iostream>

#include <glv.h>
#include <glv_binding.h>
#include <glv_util.h>


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

    //L3::SE3 pose( 100,100,0,0,0,0 );
    //L3::translate(  &cloud, &pose );

    std::pair<double,double> ll = L3::min( &cloud );
    std::pair<double,double> ur = L3::max( &cloud );


    boost::shared_ptr< L3::HistogramUniformDistance<double> > histogram( new L3::HistogramUniformDistance<double>() );
    
    double x_centre =  (ll.first + ur.first)/2.0;
    double y_centre =  (ll.second + ur.second)/2.0;
    histogram->create( x_centre, x_centre-50, x_centre + 50, 
                        y_centre, y_centre - 50, y_centre + 50 );

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
    L3::Visualisers::BasicPanController         controller( composite.position );
    
    //L3::Visualisers::HistogramVertexRenderer        histogram_vertex_renderer(histogram);
    L3::Visualisers::HistogramBoundsRenderer        histogram_bounds_renderer(histogram);
    //L3::Visualisers::HistogramDensityRenderer       histogram_density_renderer( glv::Rect(500,300), histogram );
    L3::Visualisers::HistogramVoxelRendererLeaf     histogram_voxel_renderer_leaf( histogram );
    //L3::Visualisers::HistogramVoxelRendererView     histogram_voxel_renderer_view( glv::Rect(500,0,500,300), histogram );

    composite.stretch(1,1);
    //updater->operator<< (dynamic_cast<L3::Visualisers::Updateable*>(&histogram_density_renderer));

    // Add drawables and updateables
    //top << (composite << grid << histogram_bounds_renderer << histogram_voxel_renderer_leaf << histogram_vertex_renderer) << histogram_density_renderer << histogram_voxel_renderer_view << updater.get();
    top << (composite << grid << histogram_bounds_renderer << histogram_voxel_renderer_leaf ) << updater.get();

    // Go
    win.setGLV(top);
    glv::Application::run();

}


