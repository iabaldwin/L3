#ifndef L3_VISUALISERS_LAYOUT_H
#define L3_VISUALISERS_LAYOUT_H

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "Visualisers.h"
#include "Plotters.h"

/*
 *Arbitrary text renderer
 */
namespace L3
{
namespace Visualisers
{

class Layout
{
    public:
        
        Layout( glv::Window& win ) : window(win)
        {
            
            // Create the main view
            main_view = new glv::View( glv::Rect(0,0, .6*win.width(),500));

            // Composite view holder
            composite.reset( new L3::Visualisers::Composite( glv::Rect(.6*win.width(), 500 )) );
            
            // 3D grid 
            grid.reset( new L3::Visualisers::Grid() );
           
            // Basic controller
            controller.reset( new L3::Visualisers::BasicPanController() );
        
            composite->addController( &*controller );

            // Accumulate
            (*main_view) << ( *composite << *grid );

            // Create subplots
            plot1 =  new glv::PlotFunction1D(glv::Color(0.5,0,0));
            plot1->stroke( 2.0 );

            plot_region_1 = new glv::Plot( glv::Rect( 0, 500+5, .6*window.width(), 150-5), *plot1 );
            plot_region_1->range( 0, 1000, 0 );
            plot_region_1->range( 0, 10 , 1 );

            plot2 =  new glv::PlotFunction1D(glv::Color(0,0.5,0));
            plot2->stroke( 2.0 );
            
            plot_region_2 = new glv::Plot( glv::Rect( 0, 650+5, .6*window.width(), 150-5), *plot2 );
            plot_region_2->range( 0, 1000, 0 );
            plot_region_2->range( -1, 1, 1 );


        }
        
        virtual ~Layout()
        {
        }

        void run( glv::GLV& top )
        {
            // Colors
            top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
           
            // Add core
            top << (*main_view ) << plot_region_1 << plot_region_2 ;
  
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

        glv::Window&    window; 
        glv::View*      main_view;

        glv::PlotFunction1D*    plot1;
        glv::Plot*              plot_region_1;

        glv::PlotFunction1D*    plot2;
        glv::Plot*              plot_region_2;

        std::auto_ptr<L3::Visualisers::Composite>   composite;
        std::auto_ptr<L3::Visualisers::Controller>  controller;
        std::auto_ptr<L3::Visualisers::Grid>        grid;

        std::list< glv::View* >                     renderables;

};

/*
 *  Dataset 
 */
class DatasetLayout : public Layout
{
    public:

        DatasetLayout( glv::Window& win ) : Layout(win)
        {

        }

        const L3::Dataset*                                  dataset;
        std::auto_ptr< L3::DatasetRunner >                  runner;
        std::auto_ptr< L3::Visualisers::LinearVelocityPlotter >     linear_velocity_plotter;
        std::auto_ptr< L3::Visualisers::RotationalVelocityPlotter > rotational_velocity_plotter;
        std::auto_ptr< L3::Visualisers::IteratorRenderer<L3::SE3> > iterator_renderer;

        void load( L3::Dataset* d ) 
        {
            dataset = d;

            // Start the dataset runner
            runner.reset( new L3::DatasetRunner( dataset ) );
            runner->start( dataset->start_time );

            // Create velocity plotter
            linear_velocity_plotter.reset( new L3::Visualisers::LinearVelocityPlotter( runner->LHLV_iterator.get(), plot1 ) );
            rotational_velocity_plotter.reset( new L3::Visualisers::RotationalVelocityPlotter( runner->LHLV_iterator.get(), plot2 ) );

            (*runner) << linear_velocity_plotter.get() << rotational_velocity_plotter.get();

            // Timer
            TextRenderer<double>* text_renderer = new TextRenderer<double>( runner->current_time );
            text_renderer->disable( glv::DrawBorder );
            this->renderables.push_front( text_renderer) ;
        
            iterator_renderer.reset( new L3::Visualisers::IteratorRenderer<SE3>( runner->pose_iterator.get() ) );
       
            *composite << (*iterator_renderer);
        }
        
};

/*
 *  Estimator specific
 */
class EstimatorLayout : public Layout
{
    public:

        EstimatorLayout( glv::Window& win ) : Layout(win)
        {

        }

        L3::EstimatorRunner*                runner;
        const L3::Dataset*                  dataset;

        ~EstimatorLayout()
        {
        }

        void load( L3::EstimatorRunner* runner ) 
        {
            this->runner = runner;
        }
        
};


} // Visualisers
} // L3


#endif

