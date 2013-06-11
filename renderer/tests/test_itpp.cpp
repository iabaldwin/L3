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

double add_noise( double val )
{

    val += double(rand()%10)/10.0;

    return val;
}

struct test_leaf : L3::Visualisers::Leaf
{
    test_leaf()
    {

        x.resize(1000);

        std::fill( x.begin(), x.end(), .1 );

        std::partial_sum( x.begin(), x.end(), x.begin() );

        y.resize( x.size() );
        y_filt.resize( x.size() );

        std::transform( x.begin(), x.end(), y.begin(), std::ptr_fun( sinf ) );
        std::transform( y.begin(), y.end(), y.begin(), std::ptr_fun( add_noise ) );
       
        //itpp::vec filter = "1 2 3 4";
        itpp::vec filter;
        filter.set_size(3);
        filter[0] = 1.0/3.0;
        filter[1] = 1.0/3.0;
        filter[2] = 1.0/3.0;

        itpp::vec b;
        b.set_size(1000);

        for( int i=0; i<1000; i++ )
            b[i] = y[i];

        itpp::Freq_Filt<double> FF(filter,1000);

        itpp::vec res = FF.filter( b );

        for( int i=0; i<1000; i++ )
            y_filt[i] = res[i];

    }

    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> y_filt;

    void onDraw3D(glv::GLV& g)
    {
        glv::draw::translateZ( -10);
        glv::Point3 pts[1000];
        glv::Color colors[1000];

        for( int i=0; i<1000; i++ )
            pts[i]( x[i], 10*y[i], 0 );

        glv::draw::paint( glv::draw::Points, pts, colors, 1000 );


        std::fill( colors, colors+1000, glv::Color(1,0,0 ) );
        for( int i=0; i<1000; i++ )
            pts[i]( x[i], 10*y_filt[i], 0 );
        
        glv::draw::paint( glv::draw::Lines, pts, colors, 1000 );
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


