
#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "Components.h"

#include "BulletInterface.h"


int main (int argc, char ** argv)
{

    glv::GLV top;
    glv::Window win(1400, 800, "Soaring");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    L3::Visualisers::Composite              composite;
    L3::Visualisers::BasicPanController     controller(composite.position);
    L3::Visualisers::Grid                   grid;


    boost::shared_ptr< L3::Visualisers::BulletInterface > interface( new L3::Visualisers::BulletInterface() );
    
    top << ( composite << *interface << grid );

    composite.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) ).stretch(1,1);

    win.setGLV(top);
    glv::Application::run();
}


