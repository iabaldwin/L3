#include "Layouts.h"

#include <boost/pointer_cast.hpp>

namespace L3
{
    namespace Visualisers
    {

        Layout::Layout( glv::Window& win ) : window(win)
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
            main_view->maximize();  // Maximise the view, to begin
            composite_maximise_controller.reset( new L3::Visualisers::DoubleClickMaximiseToggle( main_view ) );

            // 3D grid 
            grid.reset( new L3::Visualisers::Grid() );
            
            // Basic controller
            controller.reset( new L3::Visualisers::CompositeController( composite.get(), composite->position ) );
            
            // 3D Query - has to be after controller?
            mouse_query.reset( new L3::Visualisers::MouseQuerySelect( composite.get() ) );

            // Accumulate views
            (*main_view) << ( *composite << *grid );

            // Add synched temporal updater
            temporal_updater.reset( new Updater() );

            // Add synched spatial updater
            spatial_updater.reset( new SpatialUpdater( boost::shared_ptr< L3::PoseProvider>() ) );
            this->renderables.push_front( spatial_updater.get() );

            // Tables
            boost::shared_ptr< glv::Table > table = boost::make_shared< glv::Table >( "x,", 0, 0 );
            top << *table;
            tables.push_back( table );
        }

        DatasetLayout::DatasetLayout( glv::Window& win ) : Layout(win)
        {
            /*
             *  Stand-alone plots
             */
            addLinearVelocityPlot();
            addRotationalVelocityPlot();

            // Box 1
            ancillary_1.reset( new glv::Table("x - x , | | x, x x x, ") );

            // Runtime cloud renderer
            runtime_cloud_renderer_view.reset( new L3::Visualisers::PointCloudRendererView( glv::Rect( 360, 360 ), boost::shared_ptr< L3::PointCloud<double> >(), boost::shared_ptr<L3::SE3>() ) );
            temporal_updater->operator<<(  dynamic_cast<L3::Updateable*>(runtime_cloud_renderer_view.get() ) );
            *ancillary_1 << *runtime_cloud_renderer_view;

            point_cloud_maximise_controller.reset( new DoubleClickMaximiseToggle( runtime_cloud_renderer_view.get() ) );

            // Stand-alone scan renderer : Horizontal
            horizontal_scan_renderer.reset( new L3::Visualisers::HorizontalScanRenderer2DView( boost::shared_ptr< L3::ConstantTimeIterator< L3::LMS151 > >() , glv::Rect( 180,180 ) ) );
            temporal_updater->operator<<( horizontal_scan_renderer.get() );
            (*ancillary_1) << dynamic_cast<glv::View*>(horizontal_scan_renderer.get());
            window_controllers.push_back( boost::make_shared< DoubleClickMaximiseToggle >( dynamic_cast< glv::View* > (horizontal_scan_renderer.get() )) );

            // Stand-alone scan renderer : Vertical
            vertical_scan_renderer.reset( new L3::Visualisers::VerticalScanRenderer2DView( boost::shared_ptr< L3::ConstantTimeIterator< L3::LMS151 > >(), glv::Rect( 180,180 ) ) );
            temporal_updater->operator<<( vertical_scan_renderer.get() );
            window_controllers.push_back( boost::make_shared< DoubleClickMaximiseToggle >( dynamic_cast< glv::View* >(vertical_scan_renderer.get() ) ) );
            (*ancillary_1) << dynamic_cast<glv::View*>(vertical_scan_renderer.get());

            // Stand-alone pose renderer
            oracle_renderer.reset( new L3::Visualisers::DedicatedPoseRenderer( boost::shared_ptr<L3::PoseProvider>(), glv::Rect( 180,180 ), std::string("Estimate::INS" ) ) );
            temporal_updater->operator<<( oracle_renderer.get() );
            *ancillary_1 << *oracle_renderer;

            // Experience renderer
            experience_location.reset( new ExperienceLocationOverviewView( glv::Rect(180,180), boost::shared_ptr<L3::Experience>()  ) ); 
            window_controllers.push_back( boost::make_shared< DoubleClickMaximiseToggle >( experience_location.get() ) );
            *ancillary_1 << *experience_location;

            // Scan-matching scan renderer
            scan_matching_renderer.reset( new L3::Visualisers::ScanMatchingScanRenderer( glv::Rect( 180,180 ),boost::shared_ptr< L3::ScanMatching::Engine >() ) );
            window_controllers.push_back( boost::make_shared< DoubleClickMaximiseToggle >( dynamic_cast< glv::View* >( scan_matching_renderer.get() ) ) );
            window_controllers.push_back( boost::make_shared< DoubleClickMaximiseToggle >( dynamic_cast< glv::View* >( scan_matching_renderer->trajectory.get() ) ) );
            *ancillary_1 << *scan_matching_renderer;
       
            /*
             *  Second page table
             */

            ancillary_2.reset( new glv::Table("x ,") );

            /*
             * Text & controls
             */
            text_and_controls.reset( new glv::Table("x x,") );
            
            holder_1.reset( new glv::View( glv::Rect( 350, 100) ) );
            holder_2.reset( new glv::View( glv::Rect( 175, 100) ) );

            L3_controls.reset( new glv::Table("<,") );
          
            // Dataset scaling factor
            scale_factor.reset( new glv::Slider(glv::Rect(window.width()-155,window.height()-20,150, 10) ) );
            scale_factor->interval( 5, 1 );
           
            {
                boost::shared_ptr< glv::Label > label = boost::make_shared< glv::Label >( "Time scaling" );
                label->pos( glv::Place::CL, 5, 0 ).anchor( glv::Place::CR ); 
                *scale_factor << *label ;
                slider_labels.push_back( label );
            }

            (*L3_controls) << *scale_factor;

            // Dataset window duration
            window_duration_LIDAR.reset( new glv::Slider(glv::Rect(window.width()-155,window.height()-20,150, 10) ) );
            window_duration_LIDAR->interval( 1, 30);
            
            {
                boost::shared_ptr< glv::Label > label = boost::make_shared< glv::Label >( "LIDAR duration (s)" );
                label->pos( glv::Place::CL, 5, 0 ).anchor( glv::Place::CR ); 
                *window_duration_LIDAR << *label ;
                slider_labels.push_back( label );
            }
            
            (*L3_controls) << *window_duration_LIDAR;

            //Dataset INS duration
            window_duration_INS.reset( new glv::Slider(glv::Rect(window.width()-155,window.height()-20,150, 10) ) );
            window_duration_INS->interval( 1, 30 );
 
            {
                boost::shared_ptr< glv::Label > label = boost::make_shared< glv::Label >( "LHLV duration (s)" );
                label->pos( glv::Place::CL, 5, 0 ).anchor( glv::Place::CR ); 
                *window_duration_INS << *label ;
                slider_labels.push_back( label );
            }
            
            (*L3_controls) << *window_duration_INS;

            //Point-cloud downsample
            point_cloud_downsample.reset( new glv::Slider(glv::Rect(window.width()-155,window.height()-20,150, 10) ) );
            point_cloud_downsample->interval( 1, 10 );
            
            {
                boost::shared_ptr< glv::Label > label = boost::make_shared< glv::Label >( "Downsample factor" );
                label->pos( glv::Place::CL, 5, 0 ).anchor( glv::Place::CR ); 
                *point_cloud_downsample << *label ;
                slider_labels.push_back( label );
            }
            
            (*L3_controls) << *point_cloud_downsample;

            // Experience window
            experience_window.reset( new glv::Slider(glv::Rect(window.width()-155,window.height()-20,150, 10) ) );
            experience_window->interval( 5, 25 );
            
            {
                boost::shared_ptr< glv::Label > label = boost::make_shared< glv::Label >( "Experience nodes" );
                label->pos( glv::Place::CL, 5, 0 ).anchor( glv::Place::CR ); 
                *experience_window << *label ;
                slider_labels.push_back( label );
            }
            
            (*L3_controls) << *experience_window;

            time_renderer.reset( new TextRenderer<double>() );
            time_renderer->pos(window.width()-155, window.height()-50 );

            (*L3_controls) << *time_renderer;
          
            //L3_controls->enable( glv::DrawBorder );
            dynamic_cast< glv::Table* >(L3_controls.get())->arrange();
            dynamic_cast< glv::Table* >(L3_controls.get())->fit();

            *text_and_controls << (*holder_1 << *L3_controls );

            /*
             *  Visualisation controls
             */
            visualisation_controls.reset( new glv::Table( "x ,") );

            //Point cloud visualisation
            int width=15,height=15;
            point_cloud_visualiser_toggle   = boost::make_shared< CompositeLeafViewToggle >( boost::shared_ptr< L3::Visualisers::Leaf >(), "Runtime cloud", glv::Rect(width, height));
            point_cloud_bounds_toggle       = boost::make_shared< CompositeLeafViewToggle >( boost::shared_ptr< L3::Visualisers::Leaf >(), "Runtime bounds", glv::Rect(width, height));
            iterator_renderer_toggle        = boost::make_shared< CompositeLeafViewToggle >( boost::shared_ptr< L3::Visualisers::Leaf >(), "Window poses",  glv::Rect(width, height));
            experience_voxel_toggle         = boost::make_shared< CompositeLeafViewToggle >( boost::shared_ptr< L3::Visualisers::Leaf >(), "Experience voxel",  glv::Rect(width, height));
            experience_bounds_toggle        = boost::make_shared< CompositeLeafViewToggle >( boost::shared_ptr< L3::Visualisers::Leaf >(), "Experience bounds",  glv::Rect(width, height));

            *visualisation_controls << *point_cloud_visualiser_toggle << *point_cloud_bounds_toggle << *iterator_renderer_toggle << *experience_voxel_toggle << *experience_bounds_toggle;
            visualisation_controls->enable( glv::DrawBorder );
          
            // Default values
            iterator_renderer_toggle->setValue( true ); 

            widgets.push_back( point_cloud_visualiser_toggle );
            widgets.push_back( point_cloud_bounds_toggle);
            widgets.push_back( iterator_renderer_toggle );
            widgets.push_back( experience_voxel_toggle );
            widgets.push_back( experience_bounds_toggle );

            dynamic_cast< glv::Table* >(visualisation_controls.get())->fit();
            dynamic_cast< glv::Table* >(visualisation_controls.get())->arrange();

            *text_and_controls << ( *holder_2 << *visualisation_controls);

            dynamic_cast< glv::Table* >(text_and_controls.get())->fit();
            dynamic_cast< glv::Table* >(text_and_controls.get())->arrange();

            *ancillary_2 << *text_and_controls;


            /*
             *  Statistics
             */
            statistics.reset( new Statistics() );
            
            for( std::vector< boost::shared_ptr< StatisticsPlottable<double> > >::iterator it = statistics-> plottables.begin();
                    it != statistics->plottables.end();
                    it++)
                temporal_updater->operator<<( it->get() );

            (*ancillary_2) << *statistics;

            ancillary_2->enable( glv::DrawBorder );

            // Arrange
            dynamic_cast< glv::Table* >(ancillary_1.get())->fit();
            dynamic_cast< glv::Table* >(ancillary_1.get())->arrange();
            
            dynamic_cast< glv::Table* >(ancillary_2.get())->fit();
            dynamic_cast< glv::Table* >(ancillary_2.get())->arrange();

            (*tables[0]  ) << ancillary_1.get();
       
            boost::shared_ptr< glv::Table > table = boost::make_shared< glv::Table >( "x,", 0, 0 );
            (*table) << ancillary_2.get();
            tables.push_back( table );
            
            top << *table;
        }

        bool DatasetLayout::load( L3::DatasetRunner* runner )
        {

            /*
             *  Add the updater to the runenr
             */
            (*runner) << temporal_updater.get();

            /*
             *  Velocity plots
             */
            linear_velocity_plotter->assignIterator( runner->LHLV_iterator );
            rotational_velocity_plotter->assignIterator( runner->LHLV_iterator );

            /*
             *  Scale
             */
            runner->speedup = 1.0;
            scale_factor->attachVariable( runner->speedup );

            /*
             *  Window parameters
             */
            window_duration_LIDAR->attachVariable( runner->vertical_LIDAR->swathe_length );
            window_duration_INS->attachVariable( runner->LHLV_iterator->swathe_length );
            point_cloud_downsample->attachVariable( runner->projector->skip );

            /*
             *  Timer
             */
            time_renderer->setVariable( runner->current_time ); 

            /*
             *  Pose Iterator
             */
            composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>( iterator_renderer.get() ) );
            iterator_renderer.reset( new L3::Visualisers::IteratorRenderer<SE3>( runner->pose_iterator ) );
            *composite << (*iterator_renderer);
            iterator_renderer_toggle->leaf = iterator_renderer;

            /*
             *  Static map-view
             */
            L3::Configuration::Locale* config = L3::Configuration::LocaleFactory().getLocale( runner->mission->locale );
            
            composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>( map_view.get() ) );
            map_view = L3::Visualisers::LocaleRendererFactory::buildLocale( *config );
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(map_view.get() ) ) );

            /*
             *  Current pose estimate
             */
            for( std::deque< boost::shared_ptr< PoseRenderer > >::iterator it = pose_renderers.begin();
                    it != pose_renderers.end();
                    it++)
            {
                composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>( it->get() ) );
            }

            pose_renderers.push_back( boost::shared_ptr< L3::Visualisers::PoseRenderer>( new L3::Visualisers::PoseRenderer( *runner->current ) ) );
            pose_renderers.push_back( boost::shared_ptr< L3::Visualisers::AnimatedPoseRenderer> ( new L3::Visualisers::AnimatedPoseRenderer( *runner->current ) ) );

            for( std::deque< boost::shared_ptr< PoseRenderer > >::iterator it = pose_renderers.begin();
                    it != pose_renderers.end();
                    it++)
            {
                this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(it->get() ))); 
            }


            /*
             *  Run-time swathe
             */
            composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>( runtime_cloud_renderer_leaf.get() ) );
            runtime_cloud_renderer_leaf.reset( new L3::Visualisers::PointCloudRendererLeaf( runner->point_cloud, runner->provider ));
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(runtime_cloud_renderer_leaf.get() ) ) );
            // Toggle
            point_cloud_visualiser_toggle->leaf = runtime_cloud_renderer_leaf;

            /*
             * Run-time swathe bounds
             */
            composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>( point_cloud_bounds_renderer.get() ) );
            point_cloud_bounds_renderer.reset( new L3::Visualisers::PointCloudBoundsRenderer ( runtime_cloud_renderer_leaf->plot_cloud ) );
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(point_cloud_bounds_renderer.get() ) ) );
            point_cloud_bounds_toggle->leaf = point_cloud_bounds_renderer;

            /*
             *  Swathe cloud, separate view
             */
            runtime_cloud_renderer_view->cloud = runner->point_cloud;
            runtime_cloud_renderer_view->current_estimate = runner->current;

            /*
             *  Scan renderers
             */
            horizontal_scan_renderer->windower = runner->horizontal_LIDAR;
            vertical_scan_renderer->windower = runner->vertical_LIDAR;

            /*
             *  Scan matcher
             */
            scan_matching_renderer->engine = runner->engine;

            /*
             *  Oracle
             */
            oracle_renderer->provider = runner->provider; 
      
            /*
             *  Spatial grid
             */
            spatial_updater->provider = runner->provider;
            spatial_updater->observers.remove( (dynamic_cast<L3::SpatialObserver*>( grid.get() ) ) );
            composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>( grid.get() ) );
            grid.reset( new DynamicGrid() );
            composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>( grid.get() ) ) );
            spatial_updater->operator<< ( dynamic_cast<L3::SpatialObserver*>( grid.get() ) );
  
            /*
             *  Statistics
             */
            statistics->load( runner );
            
            /*
             *  Update view
             */

            L3::SE3 start_pose = runner->provider->operator()();

            composite->position.x = -1*start_pose.X();
            composite->position.y = -1*start_pose.Y();
        
        }

        /*
         *  Estimator layout
         */
        EstimatorLayout::EstimatorLayout( glv::Window& win) : DatasetLayout(win)
        {

            /*
             *  Histogram pyramid
             */
            pyramid_renderer.reset( new L3::Visualisers::HistogramPyramidRendererView( boost::shared_ptr< L3::HistogramPyramid<double> >(), 3 ) );

            for ( std::deque< boost::shared_ptr< HistogramDensityRenderer > >::iterator it = pyramid_renderer->renderers.begin();
                    it != pyramid_renderer->renderers.end();
                    it++ )
                temporal_updater->operator<<( it->get() );

            (*tables[0]) << *pyramid_renderer;

            histogram_bounds_renderer.reset( new L3::Visualisers::HistogramBoundsRenderer( boost::shared_ptr<L3::Histogram<double> >() ) );
            histogram_bounds_renderer->depth = -2.0 ;

            /*
             *  Cost visualisation
             */
            algorithm_costs_renderer.reset( new L3::Visualisers::AlgorithmCostRendererLeaf( boost::shared_ptr< L3::Estimator::Algorithm<double> >() ) );

            /*
             *  Experience histogram voxel
             */
            histogram_voxel_renderer_experience_leaf.reset( new L3::Visualisers::HistogramVoxelRendererLeaf( boost::shared_ptr<L3::Histogram<double> >() ) );

            /*
             *  Debug algorithm renderer
             */
            debug_algorithm_renderer.reset( new DebugAlgorithmRenderer( boost::shared_ptr< L3::Estimator::PassThrough<double> >()) );
           
        }

        bool EstimatorLayout::load( L3::EstimatorRunner* runner, boost::shared_ptr<L3::Experience> experience )
        {
            /*
             *  Do parent
             */
            DatasetLayout::load( runner );

            /*
             *  Experience & location
             */
            experience_location->experience = experience;
            experience_location->provider = runner->provider;

            /*
             *  Stand-alone pyramid renderer
             */
            pyramid_renderer->loadPyramid( experience->experience_pyramid );

            /*
             *  Histogram Bounds
             */
            composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>(histogram_bounds_renderer.get() ) );
            histogram_bounds_renderer->hist = (*experience->experience_pyramid)[0];
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(histogram_bounds_renderer.get() ) ) );
            experience_bounds_toggle->leaf = histogram_bounds_renderer;
            
            /*
             *  Histogram voxel
             */
            composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>(histogram_voxel_renderer_experience_leaf .get() ) );
            histogram_voxel_renderer_experience_leaf->hist = (*experience->experience_pyramid)[0];
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(histogram_voxel_renderer_experience_leaf.get() ))); 
            experience_voxel_toggle->leaf = histogram_voxel_renderer_experience_leaf; 
            
            /*
             *  Predicted estimates
             */
            //composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>( algorithm_costs_renderer.get() ) );
            //algorithm_costs_renderer->algorithm = runner->algorithm;
            //this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(algorithm_costs_renderer.get() ))); 
            
            /*
             *  Experience overview
             */
            experience_window->attachVariable( runner->experience->window );
  
            /*
             *  Table based algorithm renderer
             */
            algorithm_renderer = AlgorithmRendererFactory::Produce( runner->algorithm, temporal_updater.get() );

            if (algorithm_renderer)
            {
                // Already there?
                std::deque< boost::shared_ptr< glv::Table> >::iterator it  = std::find( tables.begin(), tables.end(), algorithm_renderer );

                if ( it != tables.end() )
                    tables.erase( it );
                else
                {
                    tables.push_back( algorithm_renderer );
                    top << *algorithm_renderer;
                }
            }

            /*
             *  Debug algorithm renderer - render costs,etc at INS pose
             */
            //if (!debug_algorithm_renderer->setInstance( boost::dynamic_pointer_cast< L3::Estimator::PassThrough<double> >( runner->algorithm )))
            //{
                //std::deque< boost::shared_ptr< glv::Table> >::iterator it  = std::find( tables.begin(), tables.end(), debug_algorithm_renderer );
          
                //if ( it != tables.end() )
                    //tables.erase( it );
            //}

            composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>( particle_filter_renderer.get() ) );
            particle_filter_renderer.reset( new ParticleFilterRendererLeaf( boost::dynamic_pointer_cast< L3::Estimator::ParticleFilter<double> >(runner->algorithm ) ) );
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(particle_filter_renderer.get() ))); 
        }
    }
}

