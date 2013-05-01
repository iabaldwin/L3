#ifndef L3_VISUALISERS_LAYOUT_H
#define L3_VISUALISERS_LAYOUT_H

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "Visualisers.h"
#include "Plotters.h"
#include "ExternalInterface.h"

namespace L3
{
namespace Visualisers
{

    //enum {
    //};


/*
 *  Custom GLV view
 */
struct CustomGLV : glv::GLV
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
                    this->setMouseDown(a, b, 1, 1);

                default:
                    break; 
            }

        }
    }
};

struct EventController : glv::View
{
    EventController()
    {
        this->disable( glv::Visible );
    }

	virtual bool onEvent( glv::Event::t e, glv::GLV& g)
    {
        if (e == 20)
        {
        }
    }
};

struct DataDumper : EventController
{

    DataDumper( std::list < L3::Dumpable* > dump_targets ) : targets(dump_targets)
    {

    }

    std::list < L3::Dumpable* > targets;

	bool onEvent( glv::Event::t e, glv::GLV& g)
    {

        const glv::Keyboard& k = g.keyboard();
        int key = k.key();

        // Special switch key
        if (key == 'd')
            std::for_each( targets.begin(), targets.end(), std::mem_fun( &Dumpable::dump ) );

        return true;
    }

};

struct CustomView : glv::View
{

    CustomView( const glv::Rect rect ) : glv::View(rect)
    {

    }

    std::map< glv::Event::t, std::list< glv::View*> > interfaces;

    void addGlobalInterface( glv::Event::t t, glv::View* iface )
    {
        if ( !iface )
        {
            std::cerr << "Erroneous callback" << std::cout;
            return;
        }

        std::map< glv::Event::t, std::list< glv::View*> >::iterator it = interfaces.find( t ); 

        if ( it != interfaces.end() )
            it->second.push_front(iface);
        else
        {
            std::list< glv::View*> list;
            list.push_front( iface );
            interfaces.insert( std::make_pair( t, list ) );
        }
    }



    bool onEvent( glv::Event::t e, glv::GLV& g)
    {
        std::map< glv::Event::t, std::list< glv::View*> >::iterator it = interfaces.find( e ); 

        if ( it != interfaces.end() )
            for (std::list< glv::View*>::iterator list_it = it->second.begin();
                    list_it != it->second.end();
                    list_it++ )
                (*list_it)->onEvent( e, g );
    }

};




class Layout
{
    public:
        
        Layout( glv::Window& win ) : window(win)
        {

            /*
             *  Lua interface
             */
            lua_interface.reset( new L3::Visualisers::ExternalInterface( glv::Rect(1200,800,200,150) ) ) ;
            //lua_interface.reset( new L3::Visualisers::TestInterface( glv::Rect(1200,800,200,150) ) ) ;
            //lua_interface->pos( win.width()-(200+10),win.height()-(150+10));
            this->renderables.push_front( lua_interface.get() );

            // Create the main view
            //main_view = new CustomView( glv::Rect(0,0, .6*win.width(),500));
            main_view = new glv::View( glv::Rect(0,0, .6*win.width(),500));
            this->renderables.push_front( main_view );

            // Add it is a global interface
            //main_view->addGlobalInterface( glv::Event::KeyDown, lua_interface.get() );

            // Composite view holder
            composite.reset( new L3::Visualisers::Composite( glv::Rect(.6*win.width(), 500 )) );
            
            // 3D grid 
            grid.reset( new L3::Visualisers::Grid() );
           
            // Basic controller
            controller.reset( new L3::Visualisers::BasicPanController( composite->position ) );
            composite->addController( &*controller );

            // Accumulate views
            (*main_view) << ( *composite << *grid );
       
            // Add synched updater
            updater.reset( new Updater() );
            this->renderables.push_front( updater.get() );
        }
        
        virtual ~Layout()
        {
        }

        //void run( glv::GLV& top )
        void run()
        {
  
            test_event_controller.reset( new EventController() );

            //glv::GLV top;
          
            CustomGLV top;

            top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
           
            // Add renderables provided by children
            for( std::list< glv::View* >::iterator it = renderables.begin(); it != renderables.end(); it++ )
                top << *it;

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
            plotter->drawUnderGrid(true);

            plotters.push_front( plotter );

            boost::shared_ptr< glv::Plot > plot_region( new glv::Plot( glv::Rect( 0, 650+5, .6*window.width(), 150-5), *plotter ) );

            plot_region->range( 0, 1000, 0 );
            plot_region->range( -1, 1, 1 );

            plot_region->numbering(true);
            plot_region->showNumbering(true);
        
            plots.push_front( plot_region );
            
            // Add rendererable
            this->renderables.push_front( plot_region.get() );
            // Mark as updateable
            updater->operator<<( dynamic_cast<Updateable*>(plotter.get()) );
        
            boost::shared_ptr< glv::View > velocity_label( new glv::Label("Rot. (rad/s)", true ) );
            velocity_label->pos( .6*window.width()+10, 675 );
            this->labels.push_front( velocity_label );
            this->renderables.push_front( velocity_label.get() );
             
        }


        void addLinearVelocityPlot( L3::ConstantTimeIterator< L3::LHLV >* LHLV_iterator )
        {
            /*
             *  Linear Velocity
             */
            
            // Add plotter
            boost::shared_ptr< VelocityPlotter > plotter( new L3::Visualisers::LinearVelocityPlotter( LHLV_iterator ) );
            plotter->stroke( 2.0 );
            plotter->drawUnderGrid(true);

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
        
            boost::shared_ptr< glv::View > velocity_label( new glv::Label("Lin. (m/s)", true ) );
            velocity_label->pos( .6*window.width()+10, 540 );
            this->labels.push_front( velocity_label );
            this->renderables.push_front( velocity_label.get() );
             
        }

    protected:

        //CustomView*                     main_view;
        glv::View*                      main_view;
        glv::Window&                    window; 
        //boost::shared_ptr< glv::View >  lua_interface;
        boost::shared_ptr< glv::TextView >  lua_interface;
        //boost::shared_ptr< L3::Visualisers::TestInterface >  lua_interface;

        
        std::list< glv::View* >     renderables;
        //std::list< glv::Plot* >     plottables;

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

        EstimatorLayout( glv::Window& win, L3::EstimatorRunner* runner, boost::shared_ptr<L3::Experience> experience, boost::shared_ptr< L3::PointCloud<double> > run_time_swathe ) ;

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

        boost::shared_ptr< DataDumper > dumper;
        
        boost::shared_ptr< L3::Visualisers::PointCloudRendererLeaf >        debug_renderer; 
        boost::shared_ptr< L3::Visualisers::HistogramBoundsRenderer >       debug_histogram_bounds_renderer;
        
        boost::shared_ptr< L3::Visualisers::HistogramPyramidRendererView  > pyramid_renderer;

        boost::shared_ptr< L3::Visualisers::LocaleBoundsRenderer > locale_bounds;
};


} // Visualisers
} // L3


#endif

