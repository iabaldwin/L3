#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"



int main()
{
    /*
     *Visualisation
     */
    glv::GLV top;

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    glv::Window win(1400, 800, "Visualisation::Layout");
    // Point cloud renderer
    L3::Visualisers::Composite          composite;
    L3::Visualisers::Controller*        controller = new L3::Visualisers::BasicPanController();
    L3::Visualisers::Grid               grid;

    composite.addController( controller );

    glv::View v(glv::Rect(0,0, 1000,500));
    v << ( composite << grid );

    glv::PlotFunction1D* plot1 =  new glv::PlotFunction1D(glv::Color(0.5,0,0));
    //glv::Plot plot_region_1( glv::Rect( 0, 0, win.width(), win.width()/8), *plot1 );
    //shold be v.height()
    glv::Plot plot_region_1( glv::Rect( 0, 500+5, win.width()-10, 150-5), *plot1 );


    glv::PlotFunction1D* plot2 =  new glv::PlotFunction1D(glv::Color(0.5,0,0));
    //glv::Plot plot_region_1( glv::Rect( 0, 0, win.width(), win.width()/8), *plot1 );
    //shold be v.height()
    glv::Plot plot_region_2( glv::Rect( 0, 650+5, win.width()-10, 150-5), *plot1 );



    v << plot_region_1 << plot_region_2;

    top << v;

    win.setGLV(top);
    
    glv::Application::run();

}
