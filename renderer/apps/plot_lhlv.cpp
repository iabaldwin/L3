#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"

struct VelocityPlotter : Poco::Runnable
{

    VelocityPlotter( glv::PlotFunction1D* plotter ) : data(plotter->data()), running(true)
    {
        // Assign data
        //plotter->data() = this->data;

        thread.start( *this );
    }

    bool            running;
    glv::Data&      data;
    Poco::Thread    thread;
    
    void run()
    {

        data.resize( glv::Data::DOUBLE, 1, 100 );
        //while( running )
        int runner = 0;
        while( runner++<10000  )
        {
            glv::Indexer i(data.size(1));
            
            while( i() ) 
            {
                data.assign( (double)(random() % 5 )/5.0, i[0], i[1] );
                //data.assign( (double)(1.0), i[0], i[1] );
                //data.assign( (double)(0.2), i[0], i[1] );
            }
        }

    }

};


int main (int argc, char ** argv)
{

    /*
     *Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Soaring");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    glv::Grid grid(glv::Rect(0,0));

    grid.range(1);            // set plot region
    grid.major(1);            // set major tick mark placement
    grid.minor(2);            // number of divisions per major ticks
    grid.equalizeAxes(true);
    grid.stretch(1,.2);

    double d = 800;
    glv::PlotFunction1D* plot1 =  new glv::PlotFunction1D(glv::Color(0.5,0,0));
    glv::Plot v( glv::Rect( 0, 0, d, d/8), *plot1 );
    glv::Plot w( glv::Rect( 0, d/2, d, d/8), *new glv::PlotFunction1D(glv::Color(0.5,0,0)));
   
    top << v << w;

    VelocityPlotter vel( plot1 );

    // Run
    win.setGLV(top);
    glv::Application::run();
}


