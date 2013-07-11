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
#include "LogCapture.h"
#include "DebugAlgorithmRenderer.h"
#include "AlgorithmRenderer.h"
#include "Widgets.h"
#include "DatasetController.h"
//#include "QueryInterface.h"


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
                    (*it)->pos( window.width()-(555), 0); //Here we go
                }

                window.setGLV(top);

                glv::Application::run();
            
            }

            /*
             *Velocity plots
             */
            void addLinearVelocityPlot();
            void addRotationalVelocityPlot();

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

            glv::Window& window; 
            boost::shared_ptr< glv::View > main_view;
            
            std::list< glv::View* >                         renderables;
            boost::shared_ptr< Composite>                   composite;
            std::list< boost::shared_ptr< glv::Plot > >     plots;
            std::list< boost::shared_ptr< glv::View > >     labels;

            boost::shared_ptr< LogCapture > log_capture;

            boost::shared_ptr< TableToggler > toggler;
            std::deque< boost::shared_ptr< glv::Table > > tables;
          
            boost::shared_ptr< glv::Slider >  scale_factor;
            boost::shared_ptr< glv::Slider >  window_duration_INS;
            boost::shared_ptr< glv::Slider >  window_duration;
            boost::shared_ptr< glv::Label >   window_duration_label;

            boost::shared_ptr< glv::Slider >  point_cloud_downsample;
            boost::shared_ptr< glv::Slider >  experience_window;
            
            std::list< boost::shared_ptr< EventController > > window_controllers;
            
            boost::shared_ptr< Grid>            grid;
            boost::shared_ptr< Controller>      controller;
            boost::shared_ptr< Updater >        temporal_updater;
            boost::shared_ptr< SpatialUpdater > spatial_updater;


            boost::shared_ptr< VelocityPlotter > linear_velocity_plotter_lhlv;
            boost::shared_ptr< VelocityPlotter > linear_velocity_plotter_ics;
            boost::shared_ptr< VelocityPlotter > linear_velocity_plotter_ics_unfiltered;
            boost::shared_ptr< VelocityPlotter > linear_velocity_plotter_icp;
            boost::shared_ptr< VelocityPlotter > rotational_velocity_plotter_ics;
            boost::shared_ptr< VelocityPlotter > rotational_velocity_plotter_lhlv;

            boost::shared_ptr< WASDManager > selection_manager;
            boost::shared_ptr< MouseQuerySelect > mouse_query;

            std::map< std::string, boost::shared_ptr< Leaf > > extras;

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
            
            boost::shared_ptr< PointCloudRendererLeaf >     runtime_cloud_renderer_leaf; 

            boost::shared_ptr< LocaleRenderer >             map_view;
            boost::shared_ptr< LocaleBoundsRenderer >       locale_bounds;
            std::deque< boost::shared_ptr< PoseRenderer > > pose_renderers;
          
            boost::shared_ptr< BootController >             boot_controller;

            boost::shared_ptr< HistogramPyramidRendererView  >              pyramid_renderer;
            boost::shared_ptr< PointCloudRendererView >                     runtime_cloud_renderer_view; 
            boost::shared_ptr< ScanRenderer >                               horizontal_scan_renderer;
            boost::shared_ptr< ScanRenderer >                               vertical_scan_renderer;
            boost::shared_ptr< ScanRenderer >                               horizontal_scan_renderer_leaf;
            boost::shared_ptr< ScanRenderer >                               vertical_scan_renderer_leaf;

            boost::shared_ptr< ReflectanceRenderer >                        reflectance_renderer;

            boost::shared_ptr< ScanMatchingScanRenderer >                   scan_matching_renderer;
            boost::shared_ptr< ExperienceOverviewView >                     experience_location;
            boost::shared_ptr< Reflectance >                                reflectance;
            boost::shared_ptr< DedicatedPoseRenderer>                       oracle_renderer;
            boost::shared_ptr< PointCloudBoundsRenderer >                   point_cloud_bounds_renderer;
            boost::shared_ptr< EventController >                            point_cloud_maximise_controller;
            
            boost::shared_ptr< Statistics >         statistics;
            boost::shared_ptr< FundamentalControls> fundamental_controls;
            boost::shared_ptr< PointCloudControls > point_cloud_controls;

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

            std::deque< boost::shared_ptr< glv::View > > main_view_extras;


            boost::shared_ptr< DatasetController >  dataset_controller;
            std::deque< boost::shared_ptr< glv::EventHandler > > handlers;

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

            boost::shared_ptr< L3::Experience>  experience ;

            boost::shared_ptr< AlgorithmVisualiser >                        algorithm_renderer;
            boost::shared_ptr< L3::Visualisers::DebugAlgorithmRenderer >    debug_algorithm_renderer;
            boost::shared_ptr< L3::Visualisers::AlgorithmCostRendererLeaf > algorithm_costs_renderer;
            
            boost::shared_ptr< L3::Visualisers::HistogramBoundsRenderer >       histogram_bounds_renderer;
            boost::shared_ptr< L3::Visualisers::HistogramVoxelRendererLeaf >    histogram_voxel_renderer_experience_leaf;
            
            bool load( L3::EstimatorRunner* runner, boost::shared_ptr<L3::Experience> experience );

            bool algorithm( boost::shared_ptr< L3::Estimator::Algorithm<double> > algorithm );
    };

} 

} // L3


#endif

