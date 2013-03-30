#ifndef L3_VISUALISERS_LAYOUT_H
#define L3_VISUALISERS_LAYOUT_H

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "Visualisers.h"
#include "Plotters.h"
    
template <typename T> 
struct TextRenderer : glv::View
{

    TextRenderer( T& v ) : t(v)
    {
    }

    T& t;
    
    void onDraw(glv::GLV& g)
    {
        glv::draw::color(1);
        glv::draw::lineWidth(2);

        std::stringstream ss;
        ss.precision( 15 );
        ss << t;
        glv::draw::text( ss.str().c_str() );
    }

};

namespace L3
{
namespace Visualisers
{

class Layout
{
    public:
        
        Layout( glv::Window& win ) : window(win)
        {
            composite.reset( new L3::Visualisers::Composite() );
            controller.reset( new L3::Visualisers::BasicPanController() );
            grid.reset( new L3::Visualisers::Grid() );
        
            composite->addController( &*controller );
       
            _create();
        }

        virtual ~Layout()
        {
        }

        void _create()
        {
            // Create the main view
            main_view = new glv::View( glv::Rect(0,0, 1000,500));
            (*main_view) << ( *composite << *grid );

            // Create subplots
            plot1 =  new glv::PlotFunction1D(glv::Color(0.5,0,0));
            plot_region_1 = new glv::Plot( glv::Rect( 0, 500+5, window.width()-10, 150-5), *plot1 );
            plot_region_1->range( 0,100 ); 

            plot1->stroke( 2.0 );

            plot2 =  new glv::PlotFunction1D(glv::Color(0.5,0,0));
            plot_region_2 = new glv::Plot( glv::Rect( 0, 650+5, window.width()-10, 150-5), *plot2 );
            
            plot2->stroke( 2.0 );
        }
        
        void go( glv::GLV& top )
        {
            // Colors
            top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
           
            // Add core
            top << (*main_view << plot_region_1 << plot_region_2 );
          
            // Add renderables provided by children
            std::list< glv::View* >::iterator it = renderables.begin();

            while( it != renderables.end() )
            {
                top << *it;
                it++;
            }

            window.setGLV(top);

            glv::Application::run();
        }

    protected:

        glv::Window& window; 
        glv::View* main_view;

        glv::PlotFunction1D*            plot1;
        glv::Plot*                      plot_region_1;

        glv::PlotFunction1D*            plot2;
        glv::Plot*                      plot_region_2;

        std::auto_ptr<L3::Visualisers::Composite>     composite;
        std::auto_ptr<L3::Visualisers::Controller>    controller;
        std::auto_ptr<L3::Visualisers::Grid>          grid;

        std::list< glv::View* >         renderables;

};

class DatasetLayout : public Layout, public Poco::Runnable
{
    public:

        DatasetLayout( glv::Window& win ) : Layout(win), running(true)
        {

        }

        bool            running;
        Poco::Thread    thread;          
        const L3::Dataset*                  dataset;
        std::auto_ptr< L3::DatasetRunner >  runner;
        std::auto_ptr< L3::Visualisers::VelocityPlotter > velocity_plotter;

        ~DatasetLayout()
        {
            running = false;
            if( thread.isRunning() )
                thread.join();
        }
        
        void runDataset( L3::Dataset* d ) 
        {
            dataset = d;

            // Start the dataset runner
            runner.reset( new L3::DatasetRunner( dataset ) );
            runner->start( dataset->start_time );

            // Create velocity plotter
            velocity_plotter.reset( new L3::Visualisers::VelocityPlotter( &*runner->LHLV_iterator, plot1 ) );

            // Timer
            TextRenderer<double>* text_renderer = new TextRenderer<double>( runner->current_time );
            text_renderer->disable( glv::DrawBorder );
            this->renderables.push_front( text_renderer) ;

            thread.start( *this );
        }
        
        void run()
        {
            boost::timer t;
           
            while( running )
            {
                if ( t.elapsed() > 1 )
                {
                    runner->step( .5 );
                    velocity_plotter->update();
                    t.restart();
                }
            }
        }

};

} // Visualisers
} // L3


#endif

