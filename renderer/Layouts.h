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
       
            this->renderables.push_front( main_view );
        }
        
        virtual ~Layout()
        {
        }

        void run( glv::GLV& top )
        {
            // Colors
            top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
           
            // Add core
            //top << (*main_view ) << plot_region_1 << plot_region_2;
            //top << (*main_view );
  
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
        glv::View*   main_view;

        
        boost::shared_ptr<L3::Visualisers::Composite>   composite;
        boost::shared_ptr<L3::Visualisers::Controller>  controller;
        boost::shared_ptr<L3::Visualisers::Grid>        grid;

        std::list< glv::View* >                     renderables;

};


struct Common 
{

    glv::Plottable* plot1;
    glv::Plot*      plot_region_1;

    glv::Plottable* plot2;
    glv::Plot*      plot_region_2;

};


/*
 *  Dataset 
 */
class DatasetLayout : public Layout, public Common
{
    public:

        DatasetLayout( glv::Window& win, L3::Dataset* dataset ) : Layout(win), dataset(dataset)
        {
            
            // Start the dataset runner
            runner.reset( new L3::DatasetRunner( dataset ) );
            runner->start( dataset->start_time );

            /*
             *  Linear Velocity
             */
            plot1 = new L3::Visualisers::LinearVelocityPlotter( runner->LHLV_iterator.get() );
            plot1->stroke( 2.0 );
            plot1->drawUnderGrid(true);

            plot_region_1 = new glv::Plot( glv::Rect( 0, 500+5, .6*window.width(), 150-5), *plot1 );
            plot_region_1->range( 0, 1000, 0 );
            plot_region_1->range( -1, 10 , 1 );

            plot_region_1->numbering(true);
            plot_region_1->showNumbering(true);
            
            this->renderables.push_front( plot_region_1 );

            /*
             *  Rotational Velocity
             */
            plot2 = new L3::Visualisers::RotationalVelocityPlotter( runner->LHLV_iterator.get() );
            plot2->stroke( 2.0 );

            plot_region_2 = new glv::Plot( glv::Rect( 0, 650+5, .6*window.width(), 150-5), *plot2 );
            plot_region_2->range( 0, 1000, 0 );
            plot_region_2->range( -1 , 1 , 1 );
            
            this->renderables.push_front( plot_region_2 );

            /*
             *  Timer
             */
            time_renderer.reset( new TextRenderer<double>( runner->current_time ) );
            time_renderer->pos(1200 , 10);
            
            this->renderables.push_front( time_renderer.get() );


            /*
             *  Pose Iterator
             */
            iterator_renderer.reset( new L3::Visualisers::IteratorRenderer<SE3>( runner->pose_iterator.get() ) );
            *composite << (*iterator_renderer);


            /*
             *Temporal updates
             */
            (*runner) << dynamic_cast<L3::TemporalObserver*>(plot1) << dynamic_cast<L3::TemporalObserver*>(plot2);

        }

        virtual ~DatasetLayout()
        {

        }

        const L3::Dataset*                          dataset;
        boost::shared_ptr< L3::DatasetRunner >      runner;
        boost::shared_ptr< TextRenderer<double> >   time_renderer;
        boost::shared_ptr< L3::Visualisers::IteratorRenderer<L3::SE3> > iterator_renderer;

};

/*
 *  Estimator specific
 */
class EstimatorLayout : public Layout, public Common
{
    public:

        EstimatorLayout( glv::Window& win, L3::EstimatorRunner* runner, boost::shared_ptr<L3::Experience> experience, L3::PointCloud<double>* run_time_swathe ) 
            : Layout(win), runner(runner), experience(experience)
        {
            // Histogram view
            histogram_pixel_renderer_experience.reset( new L3::Visualisers::HistogramPixelRenderer( glv::Rect(500, 300 ), experience->experience_histogram ) ) ;
            histogram_pixel_renderer_experience->pos(win.width()-510,10);
            this->renderables.push_front( histogram_pixel_renderer_experience.get() );

            histogram_bounds_renderer.reset( new L3::Visualisers::HistogramBoundsRenderer( experience->experience_histogram ) );
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(histogram_bounds_renderer.get() ) ) );

            point_cloud_bounds_renderer.reset( new L3::Visualisers::PointCloudBoundsRenderer ( run_time_swathe ) );
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(point_cloud_bounds_renderer.get() ) ) );

            runtime_cloud_renderer.reset( new L3::Visualisers::PointCloudRenderer( run_time_swathe ));
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(runtime_cloud_renderer.get() ) ) );
        }
    
        L3::EstimatorRunner* runner;
        boost::shared_ptr< L3::Experience> experience ;
            
        boost::shared_ptr< L3::Visualisers::PointCloudRenderer >        runtime_cloud_renderer; 
        boost::shared_ptr< L3::Visualisers::HistogramBoundsRenderer >   histogram_bounds_renderer;
        boost::shared_ptr< L3::Visualisers::PointCloudBoundsRenderer >  point_cloud_bounds_renderer;
        boost::shared_ptr< L3::Visualisers::HistogramPixelRenderer >    histogram_pixel_renderer_experience;

};


} // Visualisers
} // L3


#endif

