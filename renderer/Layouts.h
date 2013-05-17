#ifndef L3_VISUALISERS_LAYOUT_H
#define L3_VISUALISERS_LAYOUT_H

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "RenderCore.h"
#include "Visualisers.h"
#include "Controls.h"
#include "Plotters.h"
#include "Imagery.h"
#include "GLVInterface.h"
#include "QueryInterface.h"

namespace L3
{
namespace Visualisers
{
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
                composite->maximize();  // Maximise within the view
                main_view->maximize();  // Maximise the view

                // 3D grid 
                grid.reset( new L3::Visualisers::Grid() );

                // Basic controller
                controller.reset( new L3::Visualisers::BasicPanController( composite->position ) );
                composite->addController( &*controller );

                // Interface test
                //toggle_button.reset( new glv::Button( glv::Rect(20,20) ) );

                // 3D Query
                mouse_query.reset( new L3::Visualisers::MouseQuerySelect( composite.get() ) );
                // Action to perform on queries 
                selection_manager.reset( new L3::Visualisers::WASDManager( mouse_query.get() ) );
                
                // Accumulate views
                (*main_view) << ( *composite << *grid );

                // Add synched updater
                updater.reset( new Updater() );
                this->renderables.push_front( updater.get() );
            }

            virtual ~Layout()
            {
            }

            void run()
            {
                top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

                // Add renderables provided by children
                for( std::list< glv::View* >::iterator it = renderables.begin(); it != renderables.end(); it++ )
                    top << *it;

                composite_maximise_controller.reset( new DoubleClickMaximiseToggle( main_view ) );

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

            bool addExtra( std::string description, boost::shared_ptr< L3::Visualisers::Leaf > renderable )
            {
                // Do we already have it?
                std::map< std::string, boost::shared_ptr< L3::Visualisers::Leaf > >::iterator it = extras.find( description ); 

                if( it == extras.end() )
                {    
                    // Add it to the composite 
                    (*composite) <<  *renderable;

                    // Keep it around
                    extras.insert( std::make_pair( description, renderable ) );
               
                    return true;
                }

                return false;
            }

            bool removeExtra( std::string description )
            {
                // Do we already have it?
                std::map< std::string, boost::shared_ptr< L3::Visualisers::Leaf > >::iterator it = extras.find( description ); 

                if( it == extras.end() )
                {
                    return false;
                }
                else
                {
                    composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>( it->second.get() ) );
                    extras.erase( it );

                    return true;
                }
            }

            boost::shared_ptr< glv::View >  scripting_interface;

            boost::shared_ptr< VelocityPlotter > linear_velocity_plotter;
            boost::shared_ptr< VelocityPlotter > rotational_velocity_plotter;

        protected:

            L3GLV top;

            glv::View*      main_view;
            glv::Window&    window; 

            std::list< glv::View* > renderables;

            boost::shared_ptr<glv::Slider>  scale_factor;
            boost::shared_ptr< glv::Label > scale_factor_label;
            
            boost::shared_ptr< glv::Widget > toggle_button;

            std::list< boost::shared_ptr< glv::View > >     labels;

            boost::shared_ptr<L3::Visualisers::Composite>   composite;
            boost::shared_ptr< EventController > composite_maximise_controller;
            std::list< boost::shared_ptr< EventController > > window_controllers;
            
            boost::shared_ptr< Updater >                    updater;
            boost::shared_ptr<L3::Visualisers::Grid>        grid;
            boost::shared_ptr<L3::Visualisers::Controller>  controller;

            std::list< boost::shared_ptr< glv::Plot > >     plots;

            boost::shared_ptr< L3::Visualisers::MouseQuerySelect > mouse_query;
            boost::shared_ptr< L3::Visualisers::WASDManager > selection_manager;

            std::map< std::string, boost::shared_ptr< L3::Visualisers::Leaf > > extras;

    };


    /*
     *  Dataset 
     */
    class DatasetLayout : public Layout
    {
        public:

            DatasetLayout( glv::Window& win ) : Layout(win)
            {
                /*
                 *  Stand-alone plots
                 */
                addLinearVelocityPlot();
                addRotationalVelocityPlot();

                // Box 1
                ancillary_1.reset( new glv::Box() );
                ancillary_1->pos( window.width()-(550+5), 350+5);
                ancillary_1->fit();
                this->renderables.push_front( ancillary_1.get() );

                // Box 2
                ancillary_2.reset( new glv::Box() );
                ancillary_2->pos( window.width()-(535), 625 );
                ancillary_2->fit();
                this->renderables.push_front( ancillary_2.get() );

                // Scan-matching scan renderer
                scan_matching_renderer.reset( new L3::Visualisers::ScanMatchingScanRenderer( glv::Rect( 150,150 ),boost::shared_ptr< L3::ScanMatching::Engine >() ) );
                scan_matching_renderer->pos( 150+30, 0 ); 

                *ancillary_2 << *scan_matching_renderer;

                experience_location.reset( new ExperienceLocationOverviewView( glv::Rect(150,150), boost::shared_ptr<L3::Experience>()  ) ); 
                *ancillary_2 << *experience_location;


                // Stand-alone scan renderer : Horizontal
                horizontal_scan_renderer.reset( new L3::Visualisers::HorizontalScanRenderer2DView( boost::shared_ptr< L3::ConstantTimeIterator< L3::LMS151 > >() , glv::Rect( 182.5,175 ) ) );
                updater->operator<<( horizontal_scan_renderer.get() );
                (*ancillary_1) << dynamic_cast<glv::View*>(horizontal_scan_renderer.get());

                // Stand-alone scan renderer : Vertical
                vertical_scan_renderer.reset( new L3::Visualisers::VerticalScanRenderer2DView( boost::shared_ptr< L3::ConstantTimeIterator< L3::LMS151 > >(), glv::Rect( 182.5,175 ) ) );
                dynamic_cast<glv::View*>(vertical_scan_renderer.get())->pos( 187, 0);
                updater->operator<<( vertical_scan_renderer.get() );

                (*ancillary_1) << dynamic_cast<glv::View*>(vertical_scan_renderer.get());

                runtime_cloud_renderer_view.reset( new L3::Visualisers::PointCloudRendererView( glv::Rect( window.width()-(550+5), 0, 375-5, 350 ), boost::shared_ptr< L3::PointCloud<double> >(), boost::shared_ptr<L3::SE3>() ) );
                this->renderables.push_front( runtime_cloud_renderer_view.get() );
                updater->operator<<(  dynamic_cast<L3::Visualisers::Updateable*>(runtime_cloud_renderer_view.get() ) );

                // WHY, WHY WHY
                point_cloud_maximise_controller.reset( new DoubleClickMaximiseToggle( runtime_cloud_renderer_view.get() ) );

                // Dataset scaling factor
                scale_factor_label.reset( new glv::Label() );
                scale_factor.reset( new glv::Slider(glv::Rect(window.width()-155,window.height()-20,150, 10) ) );
                scale_factor->interval( 5, 1 );

                top << *scale_factor;


            }

            double current_time;

            const L3::Dataset*                          dataset;
            const L3::Configuration::Mission*           mission;
            boost::shared_ptr< DatasetRunner >          runner;
            boost::shared_ptr< TextRenderer<double> >   time_renderer;
            boost::shared_ptr< IteratorRenderer<L3::SE3> > iterator_renderer;
            
            boost::shared_ptr< EventController > point_cloud_maximise_controller;

            boost::shared_ptr< LocaleRenderer >                map_view;
            boost::shared_ptr< HistogramPyramidRendererView  > pyramid_renderer;

            boost::shared_ptr< PoseRenderer >           pose_renderer;
            boost::shared_ptr< LocaleBoundsRenderer >   locale_bounds;
            boost::shared_ptr< PointCloudRendererView > runtime_cloud_renderer_view; 
 
            //boost::shared_ptr< DataDumper > dumper;
            boost::shared_ptr< glv::View > ancillary_1;
            boost::shared_ptr< glv::View > ancillary_2;
            
            boost::shared_ptr< ScanRenderer2D > horizontal_scan_renderer;
            boost::shared_ptr< ScanRenderer2D > vertical_scan_renderer;
           
            boost::shared_ptr< ScanMatchingScanRenderer >  scan_matching_renderer;
    
            boost::shared_ptr< ExperienceLocationOverviewView > experience_location;
            
            /*
             *  Load/reload function
             */
            bool load( L3::DatasetRunner* runner );

    };

    /*
     *  Estimator specific
     */
    class EstimatorLayout : public DatasetLayout
    {
        public:

            EstimatorLayout( glv::Window& win) : DatasetLayout(win)
            {
            
            }

            bool load( L3::EstimatorRunner* runner, boost::shared_ptr<L3::Experience> experience );

            boost::shared_ptr< L3::Experience>                      experience ;
            boost::shared_ptr< L3::Visualisers::PoseRenderer >      estimated_pose_renderer;

            boost::shared_ptr< L3::Visualisers::DedicatedPoseRenderer> oracle_renderer;
            boost::shared_ptr< L3::Visualisers::DedicatedPoseRenderer> predicted_pose_renderer;

            boost::shared_ptr< L3::Visualisers::PredictorRenderer >             predictor_renderer;
            boost::shared_ptr< L3::Visualisers::PointCloudRendererLeaf >        runtime_cloud_renderer_leaf; 
            boost::shared_ptr< L3::Visualisers::PointCloudBoundsRenderer >      point_cloud_bounds_renderer;
            boost::shared_ptr< L3::Visualisers::HistogramVoxelRendererView >    histogram_pixel_renderer_experience_view;
            boost::shared_ptr< L3::Visualisers::HistogramVoxelRendererLeaf >    histogram_voxel_renderer_experience_leaf;


            boost::shared_ptr< L3::Visualisers::PointCloudRendererLeaf >                debug_renderer; 
            boost::shared_ptr< L3::Visualisers::HistogramBoundsRenderer >               debug_histogram_bounds_renderer;
            boost::shared_ptr< L3::Visualisers::CombinedScanRenderer2D >                combined_scan_renderer;
            std::list< boost::shared_ptr< L3::Visualisers::HistogramDensityRenderer > > density_renderers;

            boost::shared_ptr< L3::Visualisers::HistogramBoundsRenderer >       histogram_bounds_renderer;

            
            boost::shared_ptr< L3::Visualisers::CostRendererView >          cost_renderer_view;
            boost::shared_ptr< L3::Visualisers::AlgorithmCostRendererLeaf > algorithm_costs_renderer;
    };



} 

} // L3


#endif

