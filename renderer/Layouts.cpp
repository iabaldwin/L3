#include "Layouts.h"

#include <boost/pointer_cast.hpp>

namespace L3
{
    namespace Visualisers
    {
        void Layout::addLinearVelocityPlot()
        {
            /*
             *  Linear Velocity
             */
            linear_velocity_plotter_lhlv = boost::make_shared< LinearVelocityPlotter >();
            linear_velocity_plotter_lhlv->stroke( 2.0 );

            linear_velocity_plotter_ics = boost::make_shared< LinearVelocityPlotter >();
            linear_velocity_plotter_ics->stroke( 1.0 );
          
            glv::Color c( 227/255.0, 66.0/255.0, 52.0/255.0 );
            linear_velocity_plotter_ics->color( c );

            linear_velocity_plotter_icp = boost::make_shared< LinearVelocityPlotter >();
            linear_velocity_plotter_icp->stroke( 2.0 );

            // Unfiltered
            linear_velocity_plotter_ics_unfiltered = boost::make_shared< LinearVelocityPlotter >( false );
            linear_velocity_plotter_ics_unfiltered->stroke( 2.0 );

            // Add plot region
            boost::shared_ptr< glv::Plot > plot_region 
                = boost::make_shared< glv::Plot >( glv::Rect( 0, 500+5, .6*window.width(), 150-5)
                        );
                        
            plot_region->add( boost::ref( *linear_velocity_plotter_lhlv ) );
            plot_region->add( boost::ref( *linear_velocity_plotter_ics) );
            plot_region->add( boost::ref( *linear_velocity_plotter_icp ) );
            plot_region->add( boost::ref( *linear_velocity_plotter_ics_unfiltered ) );

            linear_velocity_plotter_lhlv->plot_parent = plot_region;

            plot_region->disable( glv::Controllable );
            plot_region->range( 0, 10, 0 );         // 10 s
            plot_region->range( -1, 10 , 1 );       // 10 m/s
            
            plot_region->showNumbering(true);
            plot_region->numbering(true,1);
            plot_region->numbering(false,0);

            plots.push_front( plot_region );

            // Add rendererable
            this->renderables.push_front( plot_region.get() );

            // Mark as updateable
            temporal_updater->operator<<( dynamic_cast<Updateable*>(linear_velocity_plotter_lhlv.get()) );
            temporal_updater->operator<<( dynamic_cast<Updateable*>(linear_velocity_plotter_ics.get()) );
            temporal_updater->operator<<( dynamic_cast<Updateable*>(linear_velocity_plotter_icp.get()) );
            temporal_updater->operator<<( dynamic_cast<Updateable*>(linear_velocity_plotter_ics_unfiltered.get()) );

            boost::shared_ptr< glv::View > velocity_label = boost::make_shared< glv::Label >("Linear velocity (m/s)" );
            velocity_label->pos( glv::Place::BR, 0, 0 ).anchor( glv::Place::BR );
            this->labels.push_front( velocity_label );

            *plot_region << *velocity_label;

        }
       
        void Layout::addRotationalVelocityPlot()
        {
            /*
             *  Rotational Velocity
             */

            // Add plotter
            rotational_velocity_plotter_lhlv = boost::make_shared< RotationalVelocityPlotter>();
            rotational_velocity_plotter_lhlv->stroke( 2.0 );

            glv::Color c( 119.0/255.0, 221.0/255.0, 119.0/255.0 );
            rotational_velocity_plotter_ics = boost::make_shared< RotationalVelocityPlotter>();
            rotational_velocity_plotter_ics->stroke( 1.0 );
            rotational_velocity_plotter_ics->color(c); 

            boost::shared_ptr< glv::Plot > plot_region 
                = boost::make_shared< glv::Plot >( glv::Rect( 0, 650+5, .6*window.width(), 150-5)
                        );

            plot_region->add( boost::ref( *rotational_velocity_plotter_lhlv ) );
            plot_region->add( boost::ref( *rotational_velocity_plotter_ics ) );

            rotational_velocity_plotter_lhlv->plot_parent = plot_region;

            // Scaling
            plot_region->disable( glv::Controllable );
            plot_region->range( 0, 10, 0 );
            plot_region->range( -1, 1, 1 );

            plot_region->showNumbering(true);
            plot_region->numbering(true,1);
            plot_region->numbering(false,0);

            plots.push_front( plot_region );

            // Add rendererable
            this->renderables.push_front( plot_region.get() );

            // Mark as updateable
            temporal_updater->operator<<( dynamic_cast<Updateable*>(rotational_velocity_plotter_lhlv.get()) );
            temporal_updater->operator<<( dynamic_cast<Updateable*>(rotational_velocity_plotter_ics.get()) );

            boost::shared_ptr< glv::View > velocity_label = boost::make_shared< glv::Label >("Rotational velocity. (rad/s)" );
            velocity_label->pos( glv::Place::BR, 0, 0 ).anchor( glv::Place::BR );
            this->labels.push_front( velocity_label );

            *plot_region << *velocity_label;
        }


        Layout::Layout( glv::Window& win ) : window(win)
        {
            /*
             *  Lua interface
             */
            scripting_interface = boost::make_shared< L3::Visualisers::GLVInterface >( &top );
            this->renderables.push_front( scripting_interface.get() );

            // Main view
            main_view = boost::make_shared< glv::View >( glv::Rect(0,0, .6*window.width(),500) );
            this->renderables.push_front( main_view.get() );

            // Composite view holder
            composite = boost::make_shared< L3::Visualisers::Composite >( glv::Rect(.6*window.width(), 500 ));
            composite->maximize();  // Maximise within the view
            main_view->maximize();  // Maximise the view, to begin
            window_controllers.push_back( boost::make_shared< L3::Visualisers::DoubleClickMaximiseToggle >( main_view.get() ) );

            // 3D grid 
            grid = boost::make_shared< L3::Visualisers::Grid >();
            
            // Basic controller
            controller = boost::make_shared< L3::Visualisers::CompositeController >( boost::dynamic_pointer_cast< glv::View3D >(composite).get(), boost::ref( composite->position ) );
            
            // 3D Query - has to be after controller?
            mouse_query = boost::make_shared< L3::Visualisers::MouseQuerySelect >( composite.get() );

            // Accumulate views
            (*main_view) << ( *composite << *grid );

            // Add synched temporal updater
            temporal_updater = boost::make_shared< Updater >();

            // Add synched spatial updater
            spatial_updater = boost::make_shared< SpatialUpdater >( boost::shared_ptr< L3::PoseProvider>() );
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
            ancillary_1 = boost::make_shared< glv::Table >("x - x , | | x, x x x, ");
            
            // Runtime cloud renderer
            runtime_cloud_renderer_view = boost::make_shared< L3::Visualisers::PointCloudRendererView >( glv::Rect( 360, 360 ), boost::shared_ptr< L3::PointCloud<double> >(), boost::shared_ptr<L3::SE3>() );
            temporal_updater->operator<<(  dynamic_cast<L3::Updateable*>(runtime_cloud_renderer_view.get() ) );
            *ancillary_1 << *runtime_cloud_renderer_view;

            point_cloud_maximise_controller = boost::make_shared< DoubleClickMaximiseToggle >( runtime_cloud_renderer_view.get() );

            // Stand-alone scan renderer : Horizontal
            horizontal_scan_renderer = boost::make_shared< L3::Visualisers::HorizontalScanRenderer2DView >( boost::shared_ptr< L3::ConstantTimeIterator< L3::LMS151 > >() , glv::Rect( 180,180 ) );
            temporal_updater->operator<<( horizontal_scan_renderer.get() );
            (*ancillary_1) << dynamic_cast<glv::View*>(horizontal_scan_renderer.get());
            window_controllers.push_back( boost::make_shared< DoubleClickMaximiseToggle >( dynamic_cast< glv::View* > (horizontal_scan_renderer.get() )) );

            // Stand-alone scan renderer : Vertical
            vertical_scan_renderer = boost::make_shared< L3::Visualisers::VerticalScanRenderer2DView > ( boost::shared_ptr< L3::ConstantTimeIterator< L3::LMS151 > >(), glv::Rect( 180,180 ) );
            temporal_updater->operator<<( vertical_scan_renderer.get() );
            window_controllers.push_back( boost::make_shared< DoubleClickMaximiseToggle >( dynamic_cast< glv::View* >(vertical_scan_renderer.get() ) ) );
            (*ancillary_1) << dynamic_cast<glv::View*>(vertical_scan_renderer.get());

            // Leaf scan renderer : Horizontal
            horizontal_scan_renderer_leaf = boost::make_shared< L3::Visualisers::HorizontalScanRendererLeaf >( boost::shared_ptr< L3::ConstantTimeIterator< L3::LMS151 > >(),  boost::shared_ptr<L3::PoseProvider>()  );
            temporal_updater->operator<<( horizontal_scan_renderer_leaf.get() );
            *composite << *( dynamic_cast< Leaf* > (horizontal_scan_renderer_leaf.get())) ;

            // Leaf scan renderer : Vertical
            vertical_scan_renderer_leaf = boost::make_shared< L3::Visualisers::VerticalScanRendererLeaf >( boost::shared_ptr< L3::ConstantTimeIterator< L3::LMS151 > >(),  boost::shared_ptr<L3::PoseProvider>()  );
            temporal_updater->operator<<( vertical_scan_renderer_leaf.get() );
            *composite << *( dynamic_cast< Leaf* > (vertical_scan_renderer_leaf.get())) ;

            // Stand-alone pose renderer
            oracle_renderer = boost::make_shared< L3::Visualisers::DedicatedPoseRenderer >( boost::shared_ptr<L3::PoseProvider>(), glv::Rect( 180,180 ), std::string("Estimate::INS" ) );
            temporal_updater->operator<<( oracle_renderer.get() );
            *ancillary_1 << *oracle_renderer;

            // Experience renderer
            experience_location = boost::make_shared< ExperienceOverviewView >( glv::Rect(180,180), boost::shared_ptr<L3::Experience>() );
            window_controllers.push_back( boost::make_shared< DoubleClickMaximiseToggle >( experience_location.get() ) );
            window_controllers.push_back( boost::make_shared< DoubleClickMaximiseToggle >( experience_location->sub_view.get() ) );
            *ancillary_1 << *experience_location;

            // Scan-matching scan renderer
            scan_matching_renderer = boost::make_shared< L3::Visualisers::ScanMatchingScanRenderer >( glv::Rect( 180,180 ), boost::shared_ptr< L3::ScanMatching::Engine >() );
            window_controllers.push_back( boost::make_shared< DoubleClickMaximiseToggle >( dynamic_cast< glv::View* >( scan_matching_renderer.get() ) ) );
            window_controllers.push_back( boost::make_shared< DoubleClickMaximiseToggle >( dynamic_cast< glv::View3D* >(scan_matching_renderer->trajectory.get() ) ) );
            *ancillary_1 << *scan_matching_renderer;
       
            /*
             *  Second page table
             */
            ancillary_2 = boost::make_shared< glv::Table >("x ,", 0, 5);

            /*
             * Text & controls
             */
            text_and_controls = boost::make_shared< glv::Table> ("x x,", 0, 0 );
            
            holder_1 = boost::make_shared< glv::View >( glv::Rect( 385, 100) );
            holder_2 = boost::make_shared< glv::View >( glv::Rect( 165, 100) );

            L3_controls = boost::make_shared< glv::Table >("<,", 2, 5 );
          
            // Dataset scaling factor
            scale_factor = boost::make_shared< glv::Slider > (glv::Rect(window.width()-155,window.height()-20,150, 10) );
            scale_factor->interval( 5, 1 );
           
            {
                boost::shared_ptr< glv::Label > label = boost::make_shared< glv::Label >( "Time scaling" );
                label->pos( glv::Place::CL, 5, 0 ).anchor( glv::Place::CR ); 
                *scale_factor << *label ;
                this->labels.push_back( label ); 
            }

            (*L3_controls) << *scale_factor;

            // Dataset duration
            window_duration = boost::make_shared < glv::Slider >(glv::Rect(window.width()-155,window.height()-20,150, 10) );
            {
                window_duration_label = boost::make_shared< glv::Label >( "LHLV duration (s)" );
                window_duration_label->pos( glv::Place::CL, 5, 0 ).anchor( glv::Place::CR ); 
                *window_duration << *window_duration_label ;
            }

            (*L3_controls) << *window_duration;

            //Dataset INS duration
            window_duration_INS = boost::make_shared< glv::Slider >(glv::Rect(window.width()-155,window.height()-20,150, 10) );
            window_duration_INS->interval( 1, 30 );
 
            {
                boost::shared_ptr< glv::Label > label = boost::make_shared< glv::Label >( "INS duration (s)" );
                label->pos( glv::Place::CL, 5, 0 ).anchor( glv::Place::CR ); 
                *window_duration_INS << *label ;
                this->labels.push_back( label ); 
            }
            
            (*L3_controls) << *window_duration_INS;

            //Point-cloud downsample
            point_cloud_downsample = boost::make_shared< glv::Slider >(glv::Rect(window.width()-155,window.height()-20,150, 10) );
            point_cloud_downsample->interval( 1, 10 );
            
            {
                boost::shared_ptr< glv::Label > label = boost::make_shared< glv::Label >( "Downsample factor" );
                label->pos( glv::Place::CL, 5, 0 ).anchor( glv::Place::CR ); 
                *point_cloud_downsample << *label ;
                this->labels.push_back( label ); 
            }
            
            (*L3_controls) << *point_cloud_downsample;

            // Experience window
            experience_window = boost::make_shared< glv::Slider >(glv::Rect(window.width()-155,window.height()-20,150, 10) );
            experience_window->interval( 5, 25 );
            
            {
                boost::shared_ptr< glv::Label > label = boost::make_shared< glv::Label >( "Experience nodes" );
                label->pos( glv::Place::CL, 5, 0 ).anchor( glv::Place::CR ); 
                *experience_window << *label ;
                this->labels.push_back( label ); 
            }
            
            (*L3_controls) << *experience_window;

            time_renderer = boost::make_shared<TextRenderer<double> >();
            time_renderer->pos(window.width()-155, window.height()-50 );

            (*L3_controls) << *time_renderer;
          
            dynamic_cast< glv::Table* >(L3_controls.get())->arrange();
            dynamic_cast< glv::Table* >(L3_controls.get())->fit();

            *text_and_controls << (*holder_1 << *L3_controls );

            /*
             *  Visualisation controls
             */
            visualisation_controls = boost::make_shared< glv::Table >( "x ,");

            //Point cloud visualisation
            int width=15,height=15;
            point_cloud_visualiser_toggle   = boost::make_shared< CompositeLeafViewToggle >( boost::shared_ptr< L3::Visualisers::Leaf >(), "Runtime cloud", glv::Rect(width, height));
            point_cloud_bounds_toggle       = boost::make_shared< CompositeLeafViewToggle >( boost::shared_ptr< L3::Visualisers::Leaf >(), "Runtime bounds", glv::Rect(width, height));
            iterator_renderer_toggle        = boost::make_shared< CompositeLeafViewToggle >( boost::shared_ptr< L3::Visualisers::Leaf >(), "Window poses",  glv::Rect(width, height));
            experience_voxel_toggle         = boost::make_shared< CompositeLeafViewToggle >( boost::shared_ptr< L3::Visualisers::Leaf >(), "Experience voxel",  glv::Rect(width, height));
            experience_bounds_toggle        = boost::make_shared< CompositeLeafViewToggle >( boost::shared_ptr< L3::Visualisers::Leaf >(), "Experience bounds",  glv::Rect(width, height));

            *visualisation_controls << *point_cloud_visualiser_toggle << *point_cloud_bounds_toggle << *iterator_renderer_toggle << *experience_voxel_toggle << *experience_bounds_toggle;
            visualisation_controls->disable( glv::DrawBorder );

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
            statistics = boost::make_shared< Statistics >( temporal_updater.get() );

            (*ancillary_2) << *statistics;

            /*
             *  Fundamental controls
             */

            fundamental_controls = boost::make_shared< FundamentalControls >();
            (*ancillary_2) << *fundamental_controls;

            // Arrange
            dynamic_cast< glv::Table* >(ancillary_1.get())->fit();
            dynamic_cast< glv::Table* >(ancillary_1.get())->arrange();
            
            dynamic_cast< glv::Table* >(ancillary_2.get())->fit();
            dynamic_cast< glv::Table* >(ancillary_2.get())->arrange();

            ancillary_2->fit();

            (*tables[0]  ) << ancillary_1.get();
     
            boost::shared_ptr< glv::Table > table = boost::make_shared< glv::Table >( "x,", 0, 0 );
            (*table) << ancillary_2.get();
            
            tables.push_back( table );
            top << *table;
        
       
            /*
             *  View - aware leaf, HUD essentially
             */
            // HOW?
            //boost::shared_ptr< glv::View > velocity_data = boost::make_shared< VelocityData > ( main_view.get(), boost::shared_ptr< L3::VelocityProvider >() );
            //main_view_extras.push_back( velocity_data );
     
            /*
             *  Dataset controller
             */
            dataset_controller = boost::make_shared< DatasetController >();
            main_view->addHandler( glv::Event::KeyDown, *dataset_controller );
            handlers.push_back( dataset_controller );
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
            linear_velocity_plotter_lhlv->iterator            = runner->lhlv_velocity_provider;    
            linear_velocity_plotter_icp->iterator             = runner->icp_velocity_provider;       
            linear_velocity_plotter_ics_unfiltered->iterator  = runner->ics_velocity_provider; 
            linear_velocity_plotter_ics->iterator             = runner->ics_velocity_provider;    
            
            rotational_velocity_plotter_ics->iterator         = runner->ics_velocity_provider;       
            rotational_velocity_plotter_lhlv->iterator        = runner->lhlv_velocity_provider;     

            // Bind times
            linear_velocity_plotter_lhlv->setTime(            runner->current_time );
            linear_velocity_plotter_ics->setTime(             runner->current_time );
            linear_velocity_plotter_icp->setTime(             runner->current_time );
            linear_velocity_plotter_ics_unfiltered->setTime(  runner->current_time );
            
            rotational_velocity_plotter_lhlv->setTime(        runner->current_time );
            rotational_velocity_plotter_ics->setTime(         runner->current_time );

            /*
             *  Scale
             */
            runner->speedup = 1.0;
            scale_factor->attachVariable( runner->speedup );

            /*
             *  Window parameters
             */
            if( boost::shared_ptr< L3::ConstantDistanceWindower > windower = boost::dynamic_pointer_cast< L3::ConstantDistanceWindower >( runner->pose_windower ) )
            {
                window_duration->attachVariable( windower->swathe_length );
                window_duration->interval( 10, 100 );
                window_duration_label->setValue( "Distance (m)" );
            }
            else if( boost::shared_ptr< L3::ConstantTimeWindower<L3::LHLV> > windower = boost::dynamic_pointer_cast< L3::ConstantTimeWindower<L3::LHLV> >( runner->pose_windower ) )
            {
                window_duration->interval( 1, 30);
                window_duration->attachVariable( runner->vertical_LIDAR->swathe_length );
                window_duration_label->setValue( "Time (s)" );
            }
            else
                throw std::exception();

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
            temporal_updater->operator<< (iterator_renderer.get());
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

            

            pose_renderers.push_back( boost::make_shared< L3::Visualisers::PoseRenderer>( boost::ref( *runner->estimated_pose ) ) );
            pose_renderers.push_back( boost::make_shared< L3::Visualisers::AnimatedPoseRenderer> ( boost::ref( *runner->estimated_pose ) ) );

            for( std::deque< boost::shared_ptr< PoseRenderer > >::iterator it = pose_renderers.begin();
                    it != pose_renderers.end();
                    it++)
            {
                this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(it->get() ))); 
            }

            /*
             *  Add in boot controller
             */
            composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>( boot_controller.get() ) );
            boot_controller = boost::make_shared< BootController >( boost::ref( runner->booted ) );
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(boot_controller.get() ) ) );

            for( std::deque< boost::shared_ptr< PoseRenderer > >::iterator it = pose_renderers.begin();
                    it != pose_renderers.end();
                    it++ )
            {
                boot_controller->control( it->get() );
            }

            /*
             *  Run-time swathe
             */
            composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>( runtime_cloud_renderer_leaf.get() ) );
            runtime_cloud_renderer_leaf = boost::make_shared< L3::Visualisers::PointCloudRendererLeaf >( runner->point_cloud, runner->oracle );
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(runtime_cloud_renderer_leaf.get() ) ) );
            //Toggle
            point_cloud_visualiser_toggle->leaf = runtime_cloud_renderer_leaf;

            /*
             * Run-time swathe bounds
             */
            composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>( point_cloud_bounds_renderer.get() ) );
            point_cloud_bounds_renderer = boost::make_shared< L3::Visualisers::PointCloudBoundsRenderer >( runner->point_cloud ) ;
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(point_cloud_bounds_renderer.get() ) ) );
            //Toggle
            point_cloud_bounds_toggle->leaf = point_cloud_bounds_renderer;

            /*
             *  Swathe cloud, separate view
             */
            runtime_cloud_renderer_view->cloud = runner->point_cloud;
            runtime_cloud_renderer_view->current_estimate = runner->estimated_pose;

            /*
             *  Scan renderers
             */
            horizontal_scan_renderer->windower = runner->horizontal_LIDAR;
            vertical_scan_renderer->windower = runner->vertical_LIDAR;
            
            horizontal_scan_renderer_leaf->windower = runner->horizontal_LIDAR;
            boost::dynamic_pointer_cast< HorizontalScanRendererLeaf >( horizontal_scan_renderer_leaf) ->provider = runner->oracle;
            boost::dynamic_pointer_cast< HorizontalScanRendererLeaf >( horizontal_scan_renderer_leaf)->calibration = *(runner->horizontal_projection);

            //vertical_scan_renderer_leaf->windower = runner->vertical_LIDAR;
            //boost::dynamic_pointer_cast< VerticalScanRendererLeaf >( vertical_scan_renderer_leaf) ->provider = runner->oracle;
            //boost::dynamic_pointer_cast< VerticalScanRendererLeaf >( vertical_scan_renderer_leaf)->calibration = *(runner->vertical_projection);


            /*
             *  Scan matcher
             */
            scan_matching_renderer->engine = runner->engine;
            temporal_updater->operator<< (boost::dynamic_pointer_cast< Updateable >(scan_matching_renderer->trajectory).get() );

            /*
             *  Oracle
             */
            oracle_renderer->provider = runner->provider; 
            oracle_renderer->provider = runner->oracle; 
      
            /*
             *  Spatial grid
             */
            spatial_updater->provider = runner->oracle;
            
            spatial_updater->observers.remove( (dynamic_cast<L3::SpatialObserver*>( grid.get() ) ) );
            composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>( grid.get() ) );
            grid = boost::make_shared< DynamicGrid> ();
            composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>( grid.get() ) ) );
            spatial_updater->operator<< ( dynamic_cast<L3::SpatialObserver*>( grid.get() ) );
  
            /*
             *  Statistics
             */
            statistics->load( runner );
         
            /*
             *  Fundamental controls
             */
            // Alpha/Beta
            if( boost::shared_ptr< FilteredScanMatchingVelocityProvider  > ptr = boost::dynamic_pointer_cast< FilteredScanMatchingVelocityProvider >( runner->ics_velocity_provider ) )
                fundamental_controls->associateVelocitySource( ptr ); 


            /*
             *  Associate dataset controller
             */
            dataset_controller->runner = runner;

            /*
             *  Update view
             */
            L3::SE3 start_pose = runner->oracle->operator()();

            /*
             *Init
             */
            composite->position.x = -1*start_pose.X();
            composite->position.y = -1*start_pose.Y();


            return true;
        }

        /*
         *  Estimator layout
         */
        EstimatorLayout::EstimatorLayout( glv::Window& win) : DatasetLayout(win)
        {

            /*
             *  Histogram pyramid
             */
            pyramid_renderer = boost::make_shared< L3::Visualisers::HistogramPyramidRendererView >( boost::shared_ptr< L3::HistogramPyramid<double> >(), 3 );

            for ( std::deque< boost::shared_ptr< HistogramDensityRenderer > >::iterator it = pyramid_renderer->renderers.begin();
                    it != pyramid_renderer->renderers.end();
                    it++ )
                temporal_updater->operator<<( it->get() );

            (*tables[0]) << *pyramid_renderer;

            histogram_bounds_renderer = boost::make_shared< L3::Visualisers::HistogramBoundsRenderer >( boost::shared_ptr<L3::Histogram<double> >() );
            histogram_bounds_renderer->depth = -2.0 ;

            /*
             *  Cost visualisation
             */
            algorithm_costs_renderer = boost::make_shared< L3::Visualisers::AlgorithmCostRendererLeaf >( boost::shared_ptr< L3::Estimator::Algorithm<double> >() );

            /*
             *  Experience histogram voxel
             */
            histogram_voxel_renderer_experience_leaf = boost::make_shared< L3::Visualisers::HistogramVoxelRendererLeaf >( boost::shared_ptr<L3::Histogram<double> >() );

            /*
             *  Debug algorithm renderer
             */
            debug_algorithm_renderer = boost::make_shared< DebugAlgorithmRenderer >( boost::shared_ptr< L3::Estimator::PassThrough<double> >()) ;
           
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
            experience_location->setExperience( experience );
            experience_location->setOracle( runner->oracle );
            experience_location->setEstimator( runner->algorithm );

            ReflectanceLoader loader( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/L3" );
            reflectance = loader.reflectance; 
          
            spatial_updater->observers.remove( (dynamic_cast<L3::SpatialObserver*>( reflectance.get() ) ) );
            spatial_updater->operator<< ( dynamic_cast<L3::SpatialObserver*>( reflectance.get() ) );

            //TMP
            //reflectance_renderer = boost::make_shared< ReflectanceRenderer >( reflectance );
            //composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>(reflectance_renderer.get() ) );
            //this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(reflectance_renderer.get() ) ) );
            //reflectance_renderer->pose_provider = runner->oracle;
            //</TMP
          
            experience_location->reflectance = reflectance;
            temporal_updater->operator<<( dynamic_cast<Updateable*>(experience_location.get()) );

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
             *  Experience overview
             */
            experience_window->attachVariable( runner->experience->window );
  
            /*
             *  Table based algorithm renderer
             */
            algorithm( runner->algorithm );
            
            /*
             *  Debug algorithm renderer - render costs,etc at INS pose
             */
            if (!debug_algorithm_renderer->setInstance( boost::dynamic_pointer_cast< L3::Estimator::PassThrough<double> >( runner->algorithm )))
            {
                std::deque< boost::shared_ptr< glv::Table> >::iterator it  = std::find( tables.begin(), tables.end(), debug_algorithm_renderer );
          
                if ( it != tables.end() )
                    tables.erase( it );
            }

            // Load estimator runner
            statistics->load( runner );

            return true;
        }

            
        bool EstimatorLayout::algorithm( boost::shared_ptr< L3::Estimator::Algorithm<double> > algorithm )
        {
            // Already there?
            std::deque< boost::shared_ptr< glv::Table> >::iterator it  = std::find( tables.begin(), tables.end(), algorithm_renderer );

            if ( it != tables.end() )
                tables.erase( it );
            
            // Produce algorithm renderer
            algorithm_renderer = AlgorithmRendererFactory::Produce( algorithm, temporal_updater.get(), composite );
            
            if (algorithm_renderer)
            {
                tables.push_back( algorithm_renderer );
                algorithm_renderer->pos( window.width()-(555), 0); 
                top << *algorithm_renderer;
         
                if( toggler )
                {
                    if( toggler->current_table == 2 )
                        algorithm_renderer->enable( glv::Visible );
                    else
                        algorithm_renderer->disable( glv::Visible );
                }
            
                // Associate results
                 
            }
            
            return true;
        }
    }
}

