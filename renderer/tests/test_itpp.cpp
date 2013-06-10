#include <iostream>
#include <numeric>
#include <math.h>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "itpp/signal/filter.h"
#include "itpp/signal/freq_filt.h"

#include "L3.h"
#include "Visualisers.h"
#include "Components.h"

struct test_leaf : L3::Visualisers::Leaf
{
    test_leaf()
    {

        x.resize(1000);

        std::fill( x.begin(), x.end(), 1 );

        std::partial_sum( x.begin(), x.end(), x.begin() );

        y.resize( x.size() );

        sin(2.0);

        std::transform( x.begin(), x.end(), y.begin(), std::ptr_fun( sinf ) );

    }

    std::vector<double> x;
    std::vector<double> y;

    void onDraw3D(glv::GLV& g)
    {
        glv::draw::translateZ( -400 );
        glv::Point3 pts[1000];
        glv::Color colors[1000];

        for( int i=0; i<1000; i++ )
            pts[i]( x[i], 10*y[i], 0 );

        glv::draw::paint( glv::draw::Points, pts, colors, 1000 );
    
    }
};

int main (int argc, char ** argv)
{

    glv::GLV top;
    glv::Window win(1400, 800, "Soaring");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    L3::Visualisers::Composite composite;
    test_leaf leaf;
    
    top << ( composite << leaf );

    composite.stretch(1,1);

    win.setGLV(top);
    glv::Application::run();
}


