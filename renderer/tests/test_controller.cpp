#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "Components.h"

struct test_leaf : L3::Visualisers::Leaf
{
    void onDraw3D(glv::GLV& g)
    {
        glv::Point3 pts[1000];
        glv::Color colors[1000];

        for( int i=0; i<1000; i++ )
            pts[i]( random()%100-50, random()%100-50, 0 );

        glv::draw::paint( glv::draw::Points, pts, colors, 1000 );
    
    }
};

int main (int argc, char ** argv)
{

    glv::GLV top;
    glv::Window win(1400, 800, "L3::Component");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    L3::Visualisers::Composite                  composite;
    L3::Visualisers::Grid                       grid;
    
    L3::Visualisers::CompositeController        composite_controller( &composite, composite.position );

    test_leaf leaf;
    
    top << ( composite << grid << leaf  );

    composite.stretch(1,1);
    win.setGLV(top);
    glv::Application::run();
}


