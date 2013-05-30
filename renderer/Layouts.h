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
#include "LogCapture.h"
#include "DebugAlgorithmRenderer.h"
#include "AlgorithmRenderer.h"

namespace L3
{
namespace Visualisers
{
    class Layout
    {
        public:

            Layout( glv::Window& win );

            virtual ~Layout()
            {
            }

            void run()
            {
                top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
                
                toggler = boost::make_shared< TableToggler >(  glv::Rect( window.width()-300, window.height()-20, 40, 10 ), &tables );
                top << *toggler;

                // Add renderables provided by children
                for( std::list< glv::View* >::iterator it = renderables.begin(); it != renderables.end(); it++ )
                    top << *it;

                for( std::deque< boost::shared_ptr< glv::Table > >::iterator it = tables.begin();
                        it != tables.end();
                        it++)
                {
                    (*it)->arrange();
                    (*it)->pos( window.width()-(555), 0);
                }

                window.setGLV(top);

                glv::Application::run();
            
            }

            void addRotationalVelocityPlot()
            {
                /*
                 *  Rotational Velocity
                 */

                // Add plotter
                rotational_velocity_plotter.reset( new RotationalVelocityPlotter() );
                rotational_velocity_plotter->stroke( 2.0 );

                boost::shared_ptr< glv::Plot > plot_region( new glv::Plot( glv::Rect( 0, 650+5, .6*window.width(), 150-5), *rotational_velocity_plotter ) );

                // Scaling
                plot_region->disable( glv::Controllable );
                plot_region->range( 0, 1000, 0 );
                plot_region->range( -1, 1, 1 );

                //plot_region->numbering(true);
                plot_region->showNumbering(true);

                plots.push_front( plot_region );

                // Add rendererable
                this->renderables.push_front( plot_region.get() );
                
                // Mark as updateable
                temporal_updater->operator<<( dynamic_cast<Updateable*>(rotational_velocity_plotter.get()) );

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
                linear_velocity_plotter.reset( new LinearVelocityPlotter() );
                linear_velocity_plotter->stroke( 2.0 );

                // Add plot region
                boost::shared_ptr< glv::Plot > plot_region( new glv::Plot( glv::Rect( 0, 500+5, .6*window.width(), 150-5), *linear_velocity_plotter ) );

                linear_velocity_plotter->setParent( plot_region );

                plot_region->disable( glv::Controllable );
                plot_region->range( 0, 1000, 0 );
                plot_region->range( -1, 10 , 1 );

                //plot_region->numbering(true);
                plot_region->showNumbering(true);
                plot_region->numbering(false,0);

                plots.push_front( plot_region );

                // Add rendererable
                this->renderables.push_front( plot_region.get() );
                
                // Mark as updateable
                temporal_updater->operator<<( dynamic_cast<Updateable*>(linear_velocity_plotter.get()) );

                boost::shared_ptr< glv::View > velocity_label( new glv::Label("Linear velocity (m/s)" ) );
                velocity_label->pos( glv::Place::BR, 0, 0 ).anchor( glv::Place::BR );
                //velocity_label->pos(675, 130 );
                this->labels.push_front( velocity_label );

                *plot_region << *velocity_label;
            }

            bool addExtra( std::string description, boost::shared_ptr< Leaf > renderable )
            {
                // Do we already have it?
                std::map< std::string, boost::shared_ptr< Leaf > >::iterator it = extras.find( description ); 

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
                std::map< std::string, boost::shared_ptr< Leaf > >::iterator it = extras.find( description ); 

                if( it == extras.end() )
                {
                    return false;
                }
                else
                {
                    composite->components.remove( dynamic_cast<Leaf*>( it->second.get() ) );
                    extras.erase( it );

                    return true;
                }
            }

            boost::shared_ptr< glv::View >  scripting_interface;

        protected:

            L3GLV top;

            glv::View*      main_view;
            glv::Window&    window; 
            
            std::list< glv::View* > renderables;

            boost::shared_ptr< LogCapture > log_capture;

            boost::shared_ptr< TableToggler > toggler;
            std::deque< boost::shared_ptr< glv::Table > > tables;
          
            boost::shared_ptr< glv::Slider >  scale_factor;
            boost::shared_ptr< glv::Slider >  window_duration_INS;
            boost::shared_ptr< glv::Slider >  window_duration_LIDAR;
            boost::shared_ptr< glv::Slider >  point_cloud_downsample;
            boost::shared_ptr< glv::Slider >  experience_window;
            
            std::deque< boost::shared_ptr< glv::Label > > slider_labels;
           
            std::list< boost::shared_ptr< glv::View > >     labels;

            boost::shared_ptr< Composite>           composite;
            boost::shared_ptr< EventController >    composite_maximise_controller;
            std::list< boost::shared_ptr< EventController > > window_controllers;
            
            boost::shared_ptr< Grid>            grid;
            boost::shared_ptr< Controller>      controller;
            boost::shared_ptr< Updater >        temporal_updater;
            boost::shared_ptr< SpatialUpdater > spatial_updater;

            std::list< boost::shared_ptr< glv::Plot > >     plots;

            boost::shared_ptr< VelocityPlotter > linear_velocity_plotter;
            boost::shared_ptr< VelocityPlotter > rotational_velocity_plotter;

            boost::shared_ptr< WASDManager > selection_manager;
            boost::shared_ptr< MouseQuerySelect > mouse_query;

            std::map< std::string, boost::shared_ptr< L3::Visualisers::Leaf > > extras;

    };


    /*
     *  Dataset 
     */
    class DatasetLayout : public Layout
    {
        public:

            DatasetLayout( glv::Window& win );

            const L3::Dataset*                          dataset;
            const L3::Configuration::Mission*           mission;
            boost::shared_ptr< DatasetRunner >          runner;
            
            boost::shared_ptr< TextRenderer<double> >       time_renderer;
            boost::shared_ptr< IteratorRenderer<L3::SE3> >  iterator_renderer;
            
            boost::shared_ptr< PointCloudRendererLeaf > runtime_cloud_renderer_leaf; 

            boost::shared_ptr< LocaleRenderer >             map_view;
            boost::shared_ptr< LocaleBoundsRenderer >       locale_bounds;
            std::deque< boost::shared_ptr< PoseRenderer > > pose_renderers;
            
            boost::shared_ptr< HistogramPyramidRendererView  >              pyramid_renderer;
            boost::shared_ptr< PointCloudRendererView >                     runtime_cloud_renderer_view; 
            boost::shared_ptr< ScanRenderer2D >                             horizontal_scan_renderer;
            boost::shared_ptr< ScanRenderer2D >                             vertical_scan_renderer;
            boost::shared_ptr< ScanMatchingScanRenderer >                   scan_matching_renderer;
            boost::shared_ptr< ExperienceLocationOverviewView >             experience_location;
            boost::shared_ptr< DedicatedPoseRenderer>                       oracle_renderer;
            boost::shared_ptr< L3::Visualisers::PointCloudBoundsRenderer >  point_cloud_bounds_renderer;
            boost::shared_ptr< EventController >                            point_cloud_maximise_controller;
            
            boost::shared_ptr< Statistics > statistics;

            boost::shared_ptr< glv::View > ancillary_1;
            boost::shared_ptr< glv::View > ancillary_2;
            boost::shared_ptr< glv::View > text_and_controls;
            boost::shared_ptr< glv::View > L3_controls;
            boost::shared_ptr< glv::View > visualisation_controls;
            
            boost::shared_ptr< glv::View > holder_1, holder_2;
          
            std::deque< boost::shared_ptr< glv::Widget > > widgets;
            
            boost::shared_ptr< CompositeLeafViewToggle > point_cloud_visualiser_toggle;
            boost::shared_ptr< CompositeLeafViewToggle > point_cloud_bounds_toggle;
            boost::shared_ptr< CompositeLeafViewToggle > iterator_renderer_toggle;
            boost::shared_ptr< CompositeLeafViewToggle > experience_voxel_toggle;
            boost::shared_ptr< CompositeLeafViewToggle > experience_bounds_toggle;

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

            EstimatorLayout( glv::Window& win);

            bool load( L3::EstimatorRunner* runner, boost::shared_ptr<L3::Experience> experience );

            boost::shared_ptr< L3::Experience>                      experience ;
            boost::shared_ptr< L3::Visualisers::PoseRenderer >      estimated_pose_renderer;

            boost::shared_ptr< L3::Visualisers::DedicatedPoseRenderer> predicted_pose_renderer;

            boost::shared_ptr< L3::Visualisers::PredictorRenderer >             predictor_renderer;
            boost::shared_ptr< L3::Visualisers::HistogramVoxelRendererView >    histogram_pixel_renderer_experience_view;
            boost::shared_ptr< L3::Visualisers::HistogramVoxelRendererLeaf >    histogram_voxel_renderer_experience_leaf;


            boost::shared_ptr< L3::Visualisers::PointCloudRendererLeaf >                debug_renderer; 
            boost::shared_ptr< L3::Visualisers::HistogramBoundsRenderer >               debug_histogram_bounds_renderer;
            boost::shared_ptr< L3::Visualisers::CombinedScanRenderer2D >                combined_scan_renderer;
            std::list< boost::shared_ptr< L3::Visualisers::HistogramDensityRenderer > > density_renderers;

            boost::shared_ptr< L3::Visualisers::HistogramBoundsRenderer >       histogram_bounds_renderer;
            
            boost::shared_ptr< L3::Visualisers::CostRendererView >          cost_renderer_view;
            boost::shared_ptr< L3::Visualisers::AlgorithmCostRendererLeaf > algorithm_costs_renderer;
    
            boost::shared_ptr< L3::Visualisers::DebugAlgorithmRenderer >    debug_algorithm_renderer;

            boost::shared_ptr< AlgorithmVisualiser > algorithm_renderer;
    
            boost::shared_ptr< ParticleFilterRendererLeaf> particle_filter_renderer; 
    };

} 

} // L3


#endif

