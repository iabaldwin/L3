#ifndef L3_VISUALISERS_LAYOUT_H
#define L3_VISUALISERS_LAYOUT_H

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "Visualisers.h"
#include "Clock.h"

    
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

        //str = ss.str();

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
            composite   = new L3::Visualisers::Composite();
            controller  = new L3::Visualisers::BasicPanController();
            grid        = new L3::Visualisers::Grid();
        
            composite->addController( controller );
        }

        virtual ~Layout()
        {
            delete composite;
            delete controller;
            delete grid;
            delete main_view;
        }

        glv::Window& window; 
        glv::View* main_view;

        glv::PlotFunction1D*    plot1;
        glv::Plot*              plot_region_1;

        glv::PlotFunction1D*    plot2;
        glv::Plot*              plot_region_2;

        L3::Visualisers::Composite*    composite;
        L3::Visualisers::Controller*   controller;
        L3::Visualisers::Grid*         grid;

        std::list< glv::View* > renderables;

        void run( glv::GLV& top )
        {
            // Create the main view
            main_view = new glv::View( glv::Rect(0,0, 1000,500));
            (*main_view) << ( *composite << *grid );

            // Create subplots
            plot1 =  new glv::PlotFunction1D(glv::Color(0.5,0,0));
            plot_region_1 = new glv::Plot( glv::Rect( 0, 500+5, window.width()-10, 150-5), *plot1 );

            plot2 =  new glv::PlotFunction1D(glv::Color(0.5,0,0));
            plot_region_2 = new glv::Plot( glv::Rect( 0, 650+5, window.width()-10, 150-5), *plot2 );

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
};

class DatasetLayout : public Layout
{
    public:

        DatasetLayout( glv::Window& win ) : Layout(win)
        {

        }

        L3::Visualisers::Clock clock;

        std::auto_ptr< L3::DatasetRunner > runner;

        void runDataset( L3::Dataset* d ) 
        {
            dataset = d;
   
            runner.reset( new L3::DatasetRunner( dataset ) );
            runner->start( dataset->start_time );

            TextRenderer<double>* text_renderer = new TextRenderer<double>( runner->current_time );
            this->renderables.push_front( text_renderer) ;
        
        }

        const L3::Dataset* dataset;
};

} // Visualisers
} // L3


#endif

