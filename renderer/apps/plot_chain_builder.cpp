#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"

struct Plotter 
{

    Plotter( glv::PlotFunction1D* PLOTTER, L3::TemporalObserver* observer, double time ) 
        : plotter(PLOTTER), 
        observer(observer),
        current(time)
    {
        // Run
        timer.restart();
    }

    bool            running;
    double          current;
    boost::timer    timer; 
    glv::Data       data;
    glv::PlotFunction1D*    plotter;
    L3::TemporalObserver*   observer;

    virtual ~Plotter()
    {
    }

    virtual void update() 
    {
    }
};


struct LHLVPlotter : Plotter
{

    LHLVPlotter( glv::PlotFunction1D* plotter, L3::ConstantTimeIterator< L3::LHLV >* observer, double time )  :
       Plotter( plotter, observer, time ) 
    {
        timer.restart(); 
    }

    void update()
    {
        L3::ConstantTimeIterator< L3::LHLV >*  pointer = dynamic_cast<L3::ConstantTimeIterator< L3::LHLV >* >(observer);

        pointer->update( current + timer.elapsed() );

        data.resize( glv::Data::DOUBLE, 1, pointer->window.size() );
      
        glv::Indexer i(data.size(1));

        int counter = 0;

        while( i() ) 
            data.assign( pointer->window[counter++].second->data[9], i[0], i[1] );
   
        plotter->data() = data;
    }


};


struct PlotGLV : glv::GLV
{
    void onAnimate(double dt, GLV& g)
    {
        for( std::list< Plotter* >::iterator it = plottables.begin();
                it != plottables.end();
                it++ )
        {
            (*it)->update();
        }
    }

    void add( Plotter* plotter )
    {
        plottables.push_front( plotter );
    }

    std::list< Plotter* > plottables;
};


int main (int argc, char ** argv)
{
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );

    if( !(dataset.validate() && dataset.load() ) )
        throw std::exception();

    // Constant time iterator over LHLV data
    L3::ConstantTimeIterator< L3::LHLV > iterator( dataset.LHLV_reader );

    double time = dataset.start_time;

    std::cout.precision(15);

    L3::ChainBuilder builder( &iterator );

    L3::TemporalObserver* obs = dynamic_cast<L3::TemporalObserver*>( &builder );

    /*
     *Visualisation
     */
    PlotGLV top;
    glv::Window win(1400, 800, "Apps::LHLV");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    // Velocity plotter
    glv::PlotFunction1D* plot1 = new glv::PlotFunction1D(glv::Color(0.5,0,0));
    plot1->stroke(2);

    glv::Plot v( glv::Rect( 0, 0, win.width(), win.width()/8), *plot1 );

    v.range(0, 10);

    top << v;

    LHLVPlotter vel( plot1, &iterator, dataset.start_time );

    top.add(&vel);
    
    // Run
    win.setGLV(top);
    glv::Application::run();
}


