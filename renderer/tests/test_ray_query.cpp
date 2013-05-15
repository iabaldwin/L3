
#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "Components.h"
#include "Controls.h"
#include "QueryInterface.h"
#include "Layouts.h"


int main (int argc, char ** argv)
{

    //glv::GLV top;
    L3::Visualisers::L3GLV top;

    glv::Window win(1400, 800, "Soaring");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    L3::Visualisers::Composite              composite;
    L3::Visualisers::BasicPanController     controller(composite.position);
    L3::Visualisers::Grid                   grid;
            
    L3::Visualisers::MouseQuery query(  &composite );

    // Add Boxes
    L3::Visualisers::SelectableLeaf* renderer = new L3::Visualisers::SelectableLeaf( 20 );
    L3::Visualisers::SelectableLeaf* renderer2 = new L3::Visualisers::SelectableLeaf( 5 );

    top << ( composite << grid << *renderer << *renderer2);

    composite.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) ).stretch(1,1);

    win.setGLV(top);
    glv::Application::run();
}


