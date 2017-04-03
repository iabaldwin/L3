#include <glv.h>
#include <glv_binding.h>
#include <glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "Components.h"
#include "Controls.h"

int main (int argc, char ** argv)
{
    //glv::GLV top;
    L3::Visualisers::L3GLV top;

    glv::Window win(1400, 800, "Soaring");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    L3::Visualisers::Composite              composite( glv::Rect( 1000, 600) );
    L3::Visualisers::BasicPanController     controller(composite.position);
    L3::Visualisers::Grid                   grid;
            
    boost::shared_ptr< L3::Visualisers::PointCloudRendererView > runtime_cloud_renderer_view( new L3::Visualisers::PointCloudRendererView( glv::Rect( win.width()-(550+5), 0, 375-5, 350 ), boost::shared_ptr< L3::PointCloud<double> >(), boost::shared_ptr<L3::SE3>() ) );
    
    L3::Visualisers::DoubleClickMaximiseToggle* toggle1 = new L3::Visualisers::DoubleClickMaximiseToggle( &composite );
    L3::Visualisers::DoubleClickMaximiseToggle* toggle2 = new L3::Visualisers::DoubleClickMaximiseToggle( runtime_cloud_renderer_view.get() );
    
    top << ( composite << grid ) << (*runtime_cloud_renderer_view);
    
    win.setGLV(top);
    glv::Application::run();
}


