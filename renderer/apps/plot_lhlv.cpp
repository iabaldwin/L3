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
        current = 0.0;
        // Run
        thread.start( *this );
    }

    bool            running;
    glv::Data&      data;
    Poco::Thread    thread;

    ~VelocityPlotter()
    {
        running = false;
   
        thread.join();
    }

    double current;

    void run()
    {

        //data.resize( glv::Data::DOUBLE, 100, 1 );
        data.resize( glv::Data::DOUBLE, 1, 100 );
        //while( running )
        int runner = 0;
        while( runner++<10000  )
        {
            glv::Indexer i(data.size(1));
            
            while( i() ) 
            {
                data.assign( ((double)(random() % 5 )/5.0)-1, i[0], i[1] );
            }

            usleep(.1*1e6);
        }

    }

};


int main (int argc, char ** argv)
{

    /*
     *Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Apps::LHLV");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    glv::Grid grid(glv::Rect(0,0));

    grid.range(1);            // set plot region
    grid.major(1);            // set major tick mark placement
    grid.minor(2);            // number of divisions per major ticks
    //grid.equalizeAxes(true);
    grid.stretch(1,.2);

    glv::PlotFunction1D* plot1 =  new glv::PlotFunction1D(glv::Color(0.5,0,0));
    
    glv::Plot v( glv::Rect( 0, 0, win.width(), win.width()/8), *plot1 );

    v.range(0, 100, 0);

    plot1->stroke(2);

    top << v;

    VelocityPlotter vel( plot1 );

    // Run
    win.setGLV(top);
    glv::Application::run();
}


