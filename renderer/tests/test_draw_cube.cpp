#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>


#include "L3.h"
#include "Visualisers.h"
#include "VisualiserRunner.h"
#include "Components.h"

struct Cube : L3::Visualisers::Leaf
{

    void onDraw3D( glv::GLV& g)
    {

        glv::draw::translateZ( 100 );

        L3::Visualisers::Cube cube( -5, -5, -5, 5, 5, 5, .4 );

        renderCube( &cube );
    }

};

int main (int argc, char ** argv)
{
    /*
     *Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::PointCloud");

    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    L3::Visualisers::Grid                   grid;
    L3::Visualisers::Composite              composite;
    L3::Visualisers::CompositeController    controller( &composite, composite.position );
  
    Cube c;

    composite.stretch(1,1);

    top << (composite << grid << c );

    // Go
    win.setGLV(top);
    glv::Application::run();

}


