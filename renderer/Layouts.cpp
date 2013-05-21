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
            // Chase camera
            //controller.reset( new L3::Visualisers::ChaseController( composite.get(), composite->position, *runner->current ) );
            //this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(controller.get() ) ) );
            
            // 3D Query - has to be after controller?
            mouse_query.reset( new L3::Visualisers::MouseQuerySelect( composite.get() ) );

            // Accumulate views
            (*main_view) << ( *composite << *grid );

            // Add synched temporal updater
            temporal_updater.reset( new Updater() );
            this->renderables.push_front( temporal_updater.get() );

            // Add synched spatial updater
            spatial_updater.reset( new SpatialUpdater( boost::shared_ptr< L3::PoseProvider>() ) );
            this->renderables.push_front( spatial_updater.get() );

            // Tables
            //display_table.reset( new CustomTable( "x,", 0, 0 ) );
            //boost::shared_ptr< CustomTable > table = boost::make_shared< CustomTable >( "x,", 0, 0 );
            boost::shared_ptr< glv::Table > table = boost::make_shared< glv::Table >( "x,", 0, 0 );
            top << *table;

            tables.push_back( table );

            //log_capture.reset( new LogCapture() );
            //std::cout << "HI" << std::endl;
            //std::cout << "THERE" << std::endl;
            //log_capture->bringToFront();
            //top << *log_capture;
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
            temporal_updater->operator<<(  dynamic_cast<L3::Visualisers::Updateable*>(runtime_cloud_renderer_view.get() ) );
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
            window_controllers.push_back( boost::make_shared< DoubleClickMaximiseToggle >( dynamic_cast< glv::View* >(scan_matching_renderer.get() ) ) );
            *ancillary_1 << *scan_matching_renderer;
           
            /*
             *Text & controls
             */
            ancillary_2.reset( new glv::Table("x , " ) );
            
            // Dataset scaling factor
            scale_factor.reset( new glv::Slider(glv::Rect(window.width()-155,window.height()-20,150, 10) ) );
            scale_factor->interval( 5, 1 );
            scale_factor_label.reset( new glv::Label() );
            
            (*ancillary_2) << *scale_factor;

            window_duration_LIDAR.reset( new glv::Slider(glv::Rect(window.width()-155,window.height()-20,150, 10) ) );
            window_duration_LIDAR->interval( 10, 50 );
            
            (*ancillary_2) << *window_duration_LIDAR;

            window_duration_INS.reset( new glv::Slider(glv::Rect(window.width()-155,window.height()-20,150, 10) ) );
            window_duration_INS->interval( 10, 50 );
            
            (*ancillary_2) << *window_duration_INS;

            point_cloud_downsample.reset( new glv::Slider(glv::Rect(window.width()-155,window.height()-20,150, 10) ) );
            point_cloud_downsample->interval( 1, 10 );
            
            (*ancillary_2) << *point_cloud_downsample;

            experience_window.reset( new glv::Slider(glv::Rect(window.width()-155,window.height()-20,150, 10) ) );
            experience_window->interval( 5, 25 );
            
            (*ancillary_2) << *experience_window;


            time_renderer.reset( new TextRenderer<double>() );
            time_renderer->pos(window.width()-155, window.height()-50 );

            //top << *time_renderer;
            (*ancillary_2) << *time_renderer;

            // Arrange
            dynamic_cast< glv::Table* >(ancillary_1.get())->arrange();
            dynamic_cast< glv::Table* >(ancillary_2.get())->arrange();

            ancillary_1->enable( glv::Property::DrawBorder );
            ancillary_2->enable( glv::Property::DrawBorder );

            (*tables[0]  ) << ancillary_1.get();
       
            //boost::shared_ptr< CustomTable > table = boost::make_shared< CustomTable >( "x,", 0, 0 );
            boost::shared_ptr< glv::Table > table = boost::make_shared< glv::Table >( "x,", 0, 0 );
            (*table) << ancillary_2.get();
            
            top << *table;
            tables.push_back( table );
        }

        bool DatasetLayout::load( L3::DatasetRunner* runner )
        {
            /*
             *  Velocity plots
             */
            linear_velocity_plotter->assignIterator( runner->LHLV_iterator );
            rotational_velocity_plotter->assignIterator( runner->LHLV_iterator );

            /*
             *  Scale
             */

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

            /*
             *  Static map-view
             */
            L3::Configuration::Locale* config = L3::Configuration::LocaleFactory().getLocale( runner->mission->locale );

            //// Remove it, if it is already in the composite list     
            composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>( map_view.get() ) );
            map_view = L3::Visualisers::LocaleRendererFactory::buildLocale( *config );
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(map_view.get() ) ) );

            /*
             *  Current pose estimate
             */
            composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>( pose_renderer.get() ) );
            //pose_renderer.reset( new L3::Visualisers::PoseRenderer( *runner->current ) );
            pose_renderer.reset( new L3::Visualisers::AnimatedPoseRenderer( *runner->current ) );
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(pose_renderer.get() ))); 

            /*
             *  Run-time swathe
             */
            composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>( runtime_cloud_renderer_leaf.get() ) );
            runtime_cloud_renderer_leaf.reset( new L3::Visualisers::PointCloudRendererLeaf( runner->point_cloud, runner->provider ));
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(runtime_cloud_renderer_leaf.get() ) ) );

            /*
             *Run-time swathe bounds
             */
            //point_cloud_bounds_renderer.reset( new L3::Visualisers::PointCloudBoundsRenderer ( run_time_swathe ) );
            //this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(point_cloud_bounds_renderer.get() ) ) );

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
            grid.reset( new SpatialGrid() );
            composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>( grid.get() ) ) );
            spatial_updater->operator<< ( dynamic_cast<L3::SpatialObserver*>( grid.get() ) );
    
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
             *  Debug Cost visualisation
             */
            debug_algorithm_renderer.reset( new DebugAlgorithmRenderer( boost::shared_ptr< L3::Estimator::PassThrough<double> >()) );
            
            for ( std::deque< boost::shared_ptr< HistogramDensityRenderer > >::iterator it = debug_algorithm_renderer->renderers.begin();
                    it != debug_algorithm_renderer->renderers.end();
                    it++ )
                temporal_updater->operator<<( it->get() );
            top << *debug_algorithm_renderer;
            tables.push_back( debug_algorithm_renderer );
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
             *  Histogram Bounds
             */
            composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>(histogram_bounds_renderer.get() ) );
            histogram_bounds_renderer->hist = (*experience->experience_pyramid)[0];
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(histogram_bounds_renderer.get() ) ) );

            /*
             *  Histogram voxel
             */
            composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>(histogram_voxel_renderer_experience_leaf .get() ) );
            histogram_voxel_renderer_experience_leaf->hist = (*experience->experience_pyramid)[0];
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(histogram_voxel_renderer_experience_leaf.get() ))); 

            /*
             *  Predicted estimates
             */
            composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>( algorithm_costs_renderer.get() ) );
            algorithm_costs_renderer->algorithm = runner->algorithm;
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(algorithm_costs_renderer.get() ))); 

            /*
             *  Stand-alone pyramid renderer
             */
            pyramid_renderer->loadPyramid( experience->experience_pyramid );

            /*
             *  Experience overview
             */
            experience_window->attachVariable( runner->experience->window );
       
            debug_algorithm_renderer->algorithm  = boost::dynamic_pointer_cast< L3::Estimator::PassThrough<double> >( runner->algorithm );
      
        }
    }
}

