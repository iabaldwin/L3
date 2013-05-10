#ifndef L3_VISUALISERS_LAYOUT_H
#define L3_VISUALISERS_LAYOUT_H

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "Visualisers.h"
#include "Controls.h"
#include "Plotters.h"
#include "Imagery.h"
#include "GLVInterface.h"

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
                scripting_interface.reset( new L3::Visualisers::GLVInterface( glv::Rect(1200,800,200,150) ) ) ;
                this->renderables.push_front( scripting_interface.get() );

                // Create the main view
                main_view = new glv::View( glv::Rect(0,0, .6*window.width(),500));
                this->renderables.push_front( main_view );

                // Composite view holder
                composite.reset( new L3::Visualisers::Composite( glv::Rect(.6*window.width(), 500 )) );
                composite->maximize();
                main_view->maximize();

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

            boost::shared_ptr< glv::View >  scripting_interface;

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

                composite_maximise_controller.reset( new DoubleClickMaximiseToggle( main_view) );

                window.setGLV(top);

                glv::Application::run();
            }

            void addRotationalVelocityPlot()
            {
                /*
                 *  Rotational Velocity
                 */

                // Add plotter
                rotational_velocity_plotter.reset( new L3::Visualisers::RotationalVelocityPlotter() );
                rotational_velocity_plotter->stroke( 2.0 );

                boost::shared_ptr< glv::Plot > plot_region( new glv::Plot( glv::Rect( 0, 650+5, .6*window.width(), 150-5), *rotational_velocity_plotter ) );

                // Scaling
                plot_region->range( 0, 1000, 0 );
                plot_region->range( -1, 1, 1 );

                plot_region->numbering(true);
                plot_region->showNumbering(true);

                plots.push_front( plot_region );

                // Add rendererable
                this->renderables.push_front( plot_region.get() );
                // Mark as updateable
                updater->operator<<( dynamic_cast<Updateable*>(rotational_velocity_plotter.get()) );

                boost::shared_ptr< glv::View > velocity_label( new glv::Label("Rotational velocity. (rad/s)" ) );
                velocity_label->pos( glv::Place::BR, 0, 0 ).anchor( glv::Place::BR );
                this->labels.push_front( velocity_label );

                *plot_region << *velocity_label;
            }


            void addLinearVelocityPlot()
            {
                /*
                 *  Linear Velocity
                 */

                // Add plotter
                linear_velocity_plotter.reset( new L3::Visualisers::LinearVelocityPlotter() );
                linear_velocity_plotter->stroke( 2.0 );

                // Add plot region
                boost::shared_ptr< glv::Plot > plot_region( new glv::Plot( glv::Rect( 0, 500+5, .6*window.width(), 150-5), *linear_velocity_plotter ) );

                plot_region->range( 0, 1000, 0 );
                plot_region->range( -1, 10 , 1 );

                plot_region->numbering(true);
                plot_region->showNumbering(true);

                plots.push_front( plot_region );

                // Add rendererable
                this->renderables.push_front( plot_region.get() );
                // Mark as updateable
                updater->operator<<( dynamic_cast<Updateable*>(linear_velocity_plotter.get()) );

                boost::shared_ptr< glv::View > velocity_label( new glv::Label("Linear velocity (m/s)" ) );
                velocity_label->pos( glv::Place::BR, 0, 0 ).anchor( glv::Place::BR );
                //velocity_label->pos(675, 130 );
                this->labels.push_front( velocity_label );

                *plot_region << *velocity_label;
            }

            boost::shared_ptr< VelocityPlotter > linear_velocity_plotter;
            boost::shared_ptr< VelocityPlotter > rotational_velocity_plotter;

        protected:

            glv::View*      main_view;
            glv::Window&    window; 

            std::list< glv::View* > renderables;

            boost::shared_ptr< glv::Widget > toggle_button;

            std::list< boost::shared_ptr< glv::View > >     labels;

            boost::shared_ptr< Updater >                    updater;
            boost::shared_ptr<L3::Visualisers::Grid>        grid;
            boost::shared_ptr<L3::Visualisers::Composite>   composite;
            boost::shared_ptr<L3::Visualisers::Controller>  controller;

            std::list< boost::shared_ptr< glv::Plot > >         plots;
                

            boost::shared_ptr< EventController > composite_maximise_controller;
            boost::shared_ptr< EventController > point_cloud_maximise_controller;

            std::list< boost::shared_ptr< EventController > > window_controllers;
    };


    /*
     *  Dataset 
     */
    class DatasetLayout : public Layout
    {
        public:

            DatasetLayout( glv::Window& win, L3::Dataset* dataset, L3::Configuration::Mission* mission ) 
                : Layout(win), 
                    dataset(dataset),
                    mission(mission)

            {
                // Start the dataset runner
                runner.reset( new L3::DatasetRunner( dataset, mission ) );
                runner->start();

                //addLinearVelocityPlot( runner->LHLV_iterator.get() );
                //addRotationalVelocityPlot( runner->LHLV_iterator.get() );

                addLinearVelocityPlot();
                addRotationalVelocityPlot();

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
            const L3::Configuration::Mission*           mission;
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
                this->addLinearVelocityPlot();
                this->addRotationalVelocityPlot();
            }

            bool load( L3::EstimatorRunner* runner, boost::shared_ptr<L3::Experience> experience, boost::shared_ptr< L3::PointCloud<double> > run_time_swathe );

            L3::EstimatorRunner* runner;

            boost::shared_ptr< L3::Experience>                      experience ;
            boost::shared_ptr< L3::Visualisers::PoseRenderer >      pose_renderer;
            boost::shared_ptr< L3::Visualisers::PoseRenderer >      estimated_pose_renderer;
            boost::shared_ptr< L3::Visualisers::ScanRenderer2D >    horizontal_scan_renderer;
            boost::shared_ptr< L3::Visualisers::ScanRenderer2D >    vertical_scan_renderer;

            boost::shared_ptr< L3::Visualisers::DedicatedPoseRenderer> oracle_renderer;
            boost::shared_ptr< L3::Visualisers::DedicatedPoseRenderer> predicted_pose_renderer;

            boost::shared_ptr< L3::Visualisers::PredictorRenderer >             predictor_renderer;
            boost::shared_ptr< L3::Visualisers::PointCloudRendererLeaf >        runtime_cloud_renderer_leaf; 
            boost::shared_ptr< L3::Visualisers::PointCloudRendererView >        runtime_cloud_renderer_view; 
            boost::shared_ptr< L3::Visualisers::HistogramBoundsRenderer >       histogram_bounds_renderer;
            boost::shared_ptr< L3::Visualisers::PointCloudBoundsRenderer >      point_cloud_bounds_renderer;
            boost::shared_ptr< L3::Visualisers::HistogramVoxelRendererView >    histogram_pixel_renderer_experience_view;
            boost::shared_ptr< L3::Visualisers::HistogramVoxelRendererLeaf >    histogram_voxel_renderer_experience_leaf;



            boost::shared_ptr< L3::Visualisers::LocaleRenderer>                         map_view;
            boost::shared_ptr< L3::Visualisers::PointCloudRendererLeaf >                debug_renderer; 
            boost::shared_ptr< L3::Visualisers::HistogramBoundsRenderer >               debug_histogram_bounds_renderer;
            boost::shared_ptr< L3::Visualisers::HistogramPyramidRendererView  >         pyramid_renderer;
            boost::shared_ptr< L3::Visualisers::LocaleBoundsRenderer >                  locale_bounds;
            boost::shared_ptr< L3::Visualisers::CombinedScanRenderer2D >                combined_scan_renderer;
            std::list< boost::shared_ptr< L3::Visualisers::HistogramDensityRenderer > > density_renderers;


            //boost::shared_ptr< DataDumper > dumper;
            boost::shared_ptr< glv::View > ancillary_1;
            boost::shared_ptr< glv::View > ancillary_2;

            boost::shared_ptr< L3::Visualisers::CostRendererView >          cost_renderer_view;
            boost::shared_ptr< L3::Visualisers::ScanMatchingScanRenderer >  scan_matching_renderer;
            boost::shared_ptr< L3::Visualisers::AlgorithmCostRendererLeaf > algorithm_costs_renderer;
    };

    

} 

/*
 *  Containers
 */
struct Container
{

    Container( L3::Visualisers::EstimatorLayout* layout )
    {

    }

    boost::shared_ptr< L3::Dataset >                dataset;
    boost::shared_ptr< L3::Experience >             experience;
    boost::shared_ptr< L3::EstimatorRunner >        runner;
    boost::shared_ptr< L3::Configuration::Mission > mission;

};

} // L3


#endif

