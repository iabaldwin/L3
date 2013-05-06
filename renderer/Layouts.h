#ifndef L3_VISUALISERS_LAYOUT_H
#define L3_VISUALISERS_LAYOUT_H

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "Visualisers.h"
#include "Plotters.h"
#include "ExternalInterface.h"
#include "Imagery.h"

namespace L3
{
namespace Visualisers
{

/*
 *  Custom GLV view
 */
struct L3GLV : glv::GLV
{
    bool onEvent( glv::Event::t e, glv::GLV& g)
    {
        glv::space_t a =0.0f;
        glv::space_t b =0.0f;
        
        if ( e == glv::Event::KeyDown )
        {
            const glv::Keyboard& k = g.keyboard();

            switch (k.key())
            {
                case 96:
                    this->broadcastEvent( static_cast< glv::Event::t>( 20 ) );

                    // This is quite grim
                    this->setMouseDown(a, b, 1, 1);
                    this->setMouseUp(a, b, 1, 1);
                
                case 126:
                    this->broadcastEvent( static_cast< glv::Event::t>( 21 ) );

                default:
                    break; 
            }
        }

        if ( e == glv::Event::MouseWheel )
            std::cout << "Wheel" << std::endl;
    }
};

struct Action
{

    virtual void operator()( glv::View* v )= 0;

};

struct Maximise : Action
{

    virtual void operator()( glv::View* v )
    {
        v->maximize();
        v->bringToFront();
    }

};

struct Toggle: Action
{

    Toggle() : maximised(false)
    {
    }

    bool maximised;
    
    virtual void operator()( glv::View* v )
    {
        if ( maximised )
            v->restore();
        else
            v->maximize();
   
        maximised = !maximised;
    }

};




struct EventController : glv::EventHandler
{
 
    EventController( glv::View* view ) : last_down(0.0), view(view)
    {

    }

    //Maximise action;
    Toggle action;

    L3::Timing::ChronoTimer t;

    glv::View* view;

    double last_down;
 
    virtual bool onEvent( glv::View& v, glv::GLV& g)
    {
        if (( t.elapsed() - last_down ) < .5 )
            action( view ); 

        last_down  = t.elapsed();

    }
};



//struct DataDumper : EventController
//{

    //DataDumper( std::list < L3::Dumpable* > dump_targets ) : targets(dump_targets)
    //{

    //}

    //std::list < L3::Dumpable* > targets;

	//bool onEvent( glv::Event::t e, glv::GLV& g)
    //{

        //const glv::Keyboard& k = g.keyboard();
        //int key = k.key();

        //// Special switch key
        //if (key == 'd')
            //std::for_each( targets.begin(), targets.end(), std::mem_fun( &Dumpable::dump ) );

        //return true;
    //}

//};

class Layout
{
    public:
        
        Layout( glv::Window& win ) : window(win)
        {
            /*
             *  Lua interface
             */
            lua_interface.reset( new L3::Visualisers::ExternalInterface( glv::Rect(1200,800,200,150) ) ) ;
            this->renderables.push_front( lua_interface.get() );

            // Create the main view
            main_view = new glv::View( glv::Rect(0,0, .6*window.width(),500));
            this->renderables.push_front( main_view );

            // Composite view holder
            composite.reset( new L3::Visualisers::Composite( glv::Rect(.6*window.width(), 500 )) );
          
            composite->maximize();

            // 3D grid 
            grid.reset( new L3::Visualisers::Grid() );
           
            // Basic controller
            controller.reset( new L3::Visualisers::BasicPanController( composite->position ) );
            composite->addController( &*controller );

            // Accumulate views
            (*main_view) << ( *composite << *grid );

            toggle_button.reset( new glv::Button( glv::Rect(20,20) ) );

            // Add synched updater
            updater.reset( new Updater() );
            this->renderables.push_front( updater.get() );
        }
    
        boost::shared_ptr< glv::Widget > toggle_button;

        virtual ~Layout()
        {
        }

        void run()
        {
            L3GLV top;

            top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
           
            // Add renderables provided by children
            for( std::list< glv::View* >::iterator it = renderables.begin(); it != renderables.end(); it++ )
                top << *it;

            test_event_controller.reset( new EventController( main_view ) );
            main_view->addHandler( glv::Event::MouseDown, *test_event_controller );

            window.setGLV(top);

            glv::Application::run();
        }

        void addRotationalVelocityPlot( L3::ConstantTimeIterator< L3::LHLV >* LHLV_iterator )
        {
            /*
             *  Linear Velocity
             */
            
            // Add plotter
            boost::shared_ptr< VelocityPlotter > plotter( new L3::Visualisers::RotationalVelocityPlotter( LHLV_iterator ) );
            plotter->stroke( 2.0 );

            plotters.push_front( plotter );

            boost::shared_ptr< glv::Plot > plot_region( new glv::Plot( glv::Rect( 0, 650+5, .6*window.width(), 150-5), *plotter ) );

            // Scaling
            plot_region->range( 0, 1000, 0 );
            plot_region->range( -1, 1, 1 );

            plot_region->numbering(true);
            plot_region->showNumbering(true);
        
            plots.push_front( plot_region );
            
            // Add rendererable
            this->renderables.push_front( plot_region.get() );
            // Mark as updateable
            updater->operator<<( dynamic_cast<Updateable*>(plotter.get()) );
        
            boost::shared_ptr< glv::View > velocity_label( new glv::Label("Rotational velocity. (rad/s)" ) );
            velocity_label->pos( glv::Place::BR, 0, 0 ).anchor( glv::Place::BR );
            this->labels.push_front( velocity_label );

            *plot_region << *velocity_label;
        }


        void addLinearVelocityPlot( L3::ConstantTimeIterator< L3::LHLV >* LHLV_iterator )
        {
            /*
             *  Linear Velocity
             */
            
            // Add plotter
            boost::shared_ptr< VelocityPlotter > plotter( new L3::Visualisers::LinearVelocityPlotter( LHLV_iterator ) );
            plotter->stroke( 2.0 );
            //plotter->drawUnderGrid(true);

            plotters.push_front( plotter );

            // Add plot region
            boost::shared_ptr< glv::Plot > plot_region( new glv::Plot( glv::Rect( 0, 500+5, .6*window.width(), 150-5), *plotter ) );

            plot_region->range( 0, 1000, 0 );
            plot_region->range( -1, 10 , 1 );

            plot_region->numbering(true);
            plot_region->showNumbering(true);
        
            plots.push_front( plot_region );
            
            // Add rendererable
            this->renderables.push_front( plot_region.get() );
            // Mark as updateable
            updater->operator<<( dynamic_cast<Updateable*>(plotter.get()) );
        
            boost::shared_ptr< glv::View > velocity_label( new glv::Label("Linear velocity (m/s)" ) );
            velocity_label->pos( glv::Place::BR, 0, 0 ).anchor( glv::Place::BR );
            //velocity_label->pos(675, 130 );
            this->labels.push_front( velocity_label );
             
            *plot_region << *velocity_label;
        }

    protected:

        glv::View*                      main_view;
        glv::Window&                    window; 
        boost::shared_ptr< glv::View >  lua_interface;

        std::list< glv::View* >     renderables;

        std::list< boost::shared_ptr< glv::View > >     labels;
        
        boost::shared_ptr< Updater >                    updater;
        boost::shared_ptr<L3::Visualisers::Grid>        grid;
        boost::shared_ptr<L3::Visualisers::Composite>   composite;
        boost::shared_ptr<L3::Visualisers::Controller>  controller;
        
        std::list< boost::shared_ptr< glv::Plot > >         plots;
        std::list< boost::shared_ptr< VelocityPlotter > >   plotters;

        boost::shared_ptr< EventController > test_event_controller;
};


/*
 *  Dataset 
 */
class DatasetLayout : public Layout
{
    public:

        DatasetLayout( glv::Window& win, L3::Dataset* dataset ) : Layout(win), dataset(dataset)
        {
            // Start the dataset runner
            runner.reset( new L3::DatasetRunner( dataset ) );
            runner->start( dataset->start_time );

            addLinearVelocityPlot( runner->LHLV_iterator.get() );
            addRotationalVelocityPlot( runner->LHLV_iterator.get() );
            
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
        }

        const L3::Dataset*                          dataset;
        boost::shared_ptr< L3::DatasetRunner >      runner;
        boost::shared_ptr< TextRenderer<double> >   time_renderer;
        boost::shared_ptr< L3::Visualisers::IteratorRenderer<L3::SE3> > iterator_renderer;

};

/*
 *  Estimator specific
 */
class EstimatorLayout : public Layout
{
    public:

        EstimatorLayout( glv::Window& win) : Layout(win)
        {

        }
       
        bool load( L3::EstimatorRunner* runner, boost::shared_ptr<L3::Experience> experience, boost::shared_ptr< L3::PointCloud<double> > run_time_swathe );

        L3::EstimatorRunner* runner;
            
        
        boost::shared_ptr< L3::Experience>                  experience ;
        boost::shared_ptr< L3::Visualisers::PoseRenderer >  pose_renderer;
        boost::shared_ptr< L3::Visualisers::ScanRenderer2D >  horizontal_scan_renderer;
        boost::shared_ptr< L3::Visualisers::ScanRenderer2D >  vertical_scan_renderer;

        boost::shared_ptr< L3::Visualisers::DedicatedPoseRenderer> oracle_renderer;
        boost::shared_ptr< L3::Visualisers::DedicatedPoseRenderer> predicted_pose_renderer;
        
        boost::shared_ptr< L3::Visualisers::PredictorRenderer >             predictor_renderer;
        boost::shared_ptr< L3::Visualisers::PointCloudRendererLeaf >        runtime_cloud_renderer_leaf; 
        boost::shared_ptr< L3::Visualisers::PointCloudRendererView >        runtime_cloud_renderer_view; 
        boost::shared_ptr< L3::Visualisers::HistogramBoundsRenderer >       histogram_bounds_renderer;
        boost::shared_ptr< L3::Visualisers::PointCloudBoundsRenderer >      point_cloud_bounds_renderer;
        boost::shared_ptr< L3::Visualisers::HistogramVoxelRendererView >    histogram_pixel_renderer_experience_view;
        boost::shared_ptr< L3::Visualisers::HistogramVoxelRendererLeaf >    histogram_voxel_renderer_experience_leaf;
        
        boost::shared_ptr< L3::Visualisers::CostRendererView >    cost_renderer_view;


        std::list< boost::shared_ptr< L3::Visualisers::HistogramDensityRenderer > >  density_renderers;
        boost::shared_ptr< L3::Visualisers::PointCloudRendererLeaf >        debug_renderer; 
        boost::shared_ptr< L3::Visualisers::HistogramBoundsRenderer >       debug_histogram_bounds_renderer;
        boost::shared_ptr< L3::Visualisers::HistogramPyramidRendererView  > pyramid_renderer;
        boost::shared_ptr< L3::Visualisers::LocaleBoundsRenderer > locale_bounds;
        boost::shared_ptr< L3::Visualisers::CombinedScanRenderer2D > combined_scan_renderer;

        boost::shared_ptr< L3::Visualisers::LocaleRenderer>  map_view;

        //boost::shared_ptr< DataDumper > dumper;
        boost::shared_ptr< glv::View > ancillary_1;
        boost::shared_ptr< glv::View > ancillary_2;

        boost::shared_ptr< L3::Visualisers::AlgorithmCostRendererLeaf > algorithm_costs_renderer;
};


} // Visualisers
} // L3


#endif

