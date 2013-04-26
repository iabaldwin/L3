#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "Components.h"

template <typename T>
struct Transformer : L3::Visualisers::Leaf
{
    Transformer( L3::PointCloud<T>* point_cloud ) : cloud( point_cloud )
    {
        t.restart();
    }

    L3::PointCloud<T>* cloud;
    
    boost::timer t;

    void onDraw3D( glv::GLV& g )
    {
        if ( t.elapsed() > 0.1 )
        {
            t.restart();

            L3::SE3 update( 0, -10.0, 0, 0, 0, 0 );

            translate( cloud, &update );
        }
    }

};


int main (int argc, char ** argv)
{
    boost::shared_ptr< L3::PointCloud<double> >cloud( new L3::PointCloud<double>()  );

    L3::allocate( cloud.get(), 20*1000 );

    L3::gaussianCloud( cloud.get() );

    /*
     *Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::PointCloud");

    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    L3::Visualisers::Composite          composite_view;
    L3::Visualisers::BasicPanController controller( composite_view.position );
    L3::Visualisers::Grid               grid;

    composite_view.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) ).stretch(1,1);

    // Transform tester
    Transformer<double> transformer( cloud.get() );

    // Point cloud renderer
    L3::Visualisers::PointCloudRendererLeaf cloud_renderer( cloud );

    composite_view << cloud_renderer << transformer << grid;
    top << composite_view;

    win.setGLV(top);
    glv::Application::run();
}


