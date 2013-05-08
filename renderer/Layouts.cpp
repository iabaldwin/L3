#include "Layouts.h"

namespace L3
{
    namespace Visualisers
    {

        bool EstimatorLayout::load( L3::EstimatorRunner* runner, boost::shared_ptr<L3::Experience> experience, boost::shared_ptr< L3::PointCloud<double> > run_time_swathe ) 
        {
            this->runner = runner;
            this->experience = experience;
          
            /*
             *  Composite Leafs
             */
            L3::Configuration::Begbroke begbroke;
            begbroke.loadDatum();

            // Static map-view
            map_view.reset( new L3::Visualisers::LocaleRenderer() );
            map_view->load( begbroke );
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(map_view.get() ) ) );
            
            // Histogram Bounds
            histogram_bounds_renderer.reset( new L3::Visualisers::HistogramBoundsRenderer( (*experience->experience_pyramid)[0]) );
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(histogram_bounds_renderer.get() ) ) );
            histogram_bounds_renderer->depth = -2.0 ;
            histogram_bounds_renderer->draw_bounds = true;

            // Histogram voxel
            histogram_voxel_renderer_experience_leaf.reset( new L3::Visualisers::HistogramVoxelRendererLeaf( (*experience->experience_pyramid)[0] ) ) ;
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(histogram_voxel_renderer_experience_leaf.get() ))); 

            // Swathe Bounds
            //point_cloud_bounds_renderer.reset( new L3::Visualisers::PointCloudBoundsRenderer ( run_time_swathe ) );
            //this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(point_cloud_bounds_renderer.get() ) ) );

            // Swathe Cloud
            //runtime_cloud_renderer_leaf.reset( new L3::Visualisers::PointCloudRendererLeaf( run_time_swathe ));
            //this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(runtime_cloud_renderer_leaf.get() ) ) );

            // Current pose estimate
            //pose_renderer.reset( new L3::Visualisers::PoseRenderer( *runner->current ) );
            pose_renderer.reset( new L3::Visualisers::AnimatedPoseRenderer( *runner->current ) );
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(pose_renderer.get() ))); 

            // Estimated pose
            estimated_pose_renderer.reset( new L3::Visualisers::PoseRenderer( *runner->estimated ) );
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(estimated_pose_renderer.get() ))); 

            // Predicted estimates
            algorithm_costs_renderer.reset( new L3::Visualisers::AlgorithmCostRendererLeaf( dynamic_cast<L3::Estimator::IterativeDescent<double>* >( runner->estimator) ));
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(algorithm_costs_renderer.get() ))); 
            algorithm_costs_renderer->draw_bounds = true;

            //Locale Bounds
            locale_bounds.reset( new L3::Visualisers::LocaleBoundsRenderer() );
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(locale_bounds.get() ))); 

            /*
             *  Stand-alone plots
             */
            // Velocity plots
            addLinearVelocityPlot( runner->windower->constant_time_iterator );
            addRotationalVelocityPlot( runner->windower->constant_time_iterator );

            /*
             *  Pyramid Renderer
             */
            pyramid_renderer.reset( new L3::Visualisers::HistogramPyramidRendererView(  glv::Rect( 150*3, 150 ), experience->experience_pyramid) );
            this->renderables.push_front( pyramid_renderer.get() );

            for ( std::list< boost::shared_ptr< HistogramDensityRenderer > >::iterator it = pyramid_renderer->renderers.begin();
                    it != pyramid_renderer->renderers.end();
                    it++ )
            updater->operator<<( it->get() );
            pyramid_renderer->pos( window.width() - (175+5), 0 );

            /*
             *  Swathe Cloud
             */
            //runtime_cloud_renderer_view.reset( new L3::Visualisers::PointCloudRendererView( glv::Rect( window.width()-(525+10), 190, 525, 250 ), run_time_swathe, runner->current ));
            runtime_cloud_renderer_view.reset( new L3::Visualisers::PointCloudRendererView( glv::Rect( window.width()-(550+5), 0, 375-5, 350 ), run_time_swathe, runner->current ));
            this->renderables.push_front( runtime_cloud_renderer_view.get() );
            updater->operator<<( runtime_cloud_renderer_view.get() );

            point_cloud_maximise_controller.reset( new EventController( runtime_cloud_renderer_view.get(), glv::Event::MouseDown) );

            /*
             *  Group: Ancillary
             */
            ancillary_1.reset( new glv::Box() );

            // Stand-alone scan renderer :: Horizontal
            horizontal_scan_renderer.reset( new L3::Visualisers::HorizontalScanRenderer2DView( runner->horizontal_LIDAR, glv::Rect( 175,175 ) ) );
            updater->operator<<( horizontal_scan_renderer.get() );

            (*ancillary_1) << dynamic_cast<glv::View*>(horizontal_scan_renderer.get());

            // Stand-alone scan renderer : Vertical
            vertical_scan_renderer.reset( new L3::Visualisers::VerticalScanRenderer2DView( runner->vertical_LIDAR, glv::Rect( 175,175 ) ) );
            dynamic_cast<glv::View*>(vertical_scan_renderer.get())->pos( 175+5, 0);
            updater->operator<<( vertical_scan_renderer.get() );

            (*ancillary_1) << dynamic_cast<glv::View*>(vertical_scan_renderer.get());

            //combined_scan_renderer.reset( new L3::Visualisers::CombinedScanRenderer2D(  runner->horizontal_LIDAR, runner->vertical_LIDAR, glv::Rect(150,150) ) );
            //combined_scan_renderer->pos( 150*2+2*30, 0 );
            //for( std::list< boost::shared_ptr< ScanRenderer2D > >::iterator it = combined_scan_renderer->scan_renderers.begin();
                    //it != combined_scan_renderer->scan_renderers.end();
                    //it++ )
            //updater->operator<<( it->get() );

            //(*ancillary_1) << combined_scan_renderer.get();

            // Add it to the view
            ancillary_1->pos( window.width()-(550+5), 350+5);
            ancillary_1->fit();
            this->renderables.push_front( ancillary_1.get() );

            /*
             *  Ancillary 2
             */
            ancillary_2.reset( new glv::Box() );

            // Stand-alone pose renderer
            oracle_renderer.reset( new L3::Visualisers::DedicatedPoseRenderer( runner->provider, glv::Rect( 150,150 ), std::string("Estimate::INS" ) ) );
            updater->operator<<( oracle_renderer.get() );

            // Scan matching scan renderer
            scan_matching_renderer.reset( new L3::Visualisers::ScanMatchingScanRenderer( glv::Rect( 150,150 ),runner->engine ) );
            scan_matching_renderer->pos( 150+30, 0 ); 

            boost::shared_ptr< L3::Visualisers::EventController > controller = boost::make_shared< L3::Visualisers::EventController>( scan_matching_renderer.get(), glv::Event::MouseDown  );
            window_controllers.push_front( controller ); 

            ancillary_2->pos( window.width()-(535), 625 );
            ancillary_2->fit();

            // Add it to the view
            this->renderables.push_front( ancillary_2.get() );
            (*ancillary_2) << dynamic_cast<glv::View*>(oracle_renderer.get());
            (*ancillary_2) << dynamic_cast<glv::View*>(scan_matching_renderer.get());

            boost::shared_ptr< L3::Visualisers::EventController > tmp_controller = boost::make_shared< L3::Visualisers::EventController>( ancillary_2.get(), glv::Event::MouseDown  );
            window_controllers.push_front( tmp_controller ); 

            /*
             *  Cost visualisation 
             */
            //cost_renderer_view.reset( new L3::Visualisers::CostRendererView( *(runner->estimator->pose_estimates),  glv::Rect( win.width()-510,  win.height()-250, 200, 200 ) ) );
            //this->renderables.push_front( cost_renderer_view.get() );

            //dumper.reset( new DataDumper( runner->dumps ) );
            //main_view->addGlobalInterface( glv::Event::KeyDown, dumper.get() );
       
            // Truly dbg
            //debug_renderer.reset( new L3::Visualisers::PointCloudRendererLeaf( runner->estimator->current_swathe ));
            //this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(debug_renderer.get() ) ) );
            //debug_renderer->color = glv::Color( 160, 32, 240 ); 

            //debug_histogram_bounds_renderer.reset( new L3::Visualisers::HistogramBoundsRenderer( runner->estimator->current_histogram ) );
            //this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(debug_histogram_bounds_renderer.get() ) ) );
            //debug_histogram_bounds_renderer->depth = -12.0;
        }
    }
}
