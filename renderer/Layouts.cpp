#include "Layouts.h"

#include <boost/pointer_cast.hpp>

namespace L3
{
    namespace Visualisers
    {

        DatasetLayout::DatasetLayout( glv::Window& win ) : Layout(win)
        {
            /*
             *  Stand-alone plots
             */
            addLinearVelocityPlot();
            addRotationalVelocityPlot();

            // Box 1
            //ancillary_1.reset( new glv::Table("x | | , x x x,") );
            //ancillary_1.reset( new glv::Table("x . x , | | x,") );
            ancillary_1.reset( new glv::Table("x - x , | | x, x x x, ") );
            ancillary_1->pos( window.width()-(555), 5);
            this->renderables.push_front( ancillary_1.get() );

            // Runtime cloud renderer
            runtime_cloud_renderer_view.reset( new L3::Visualisers::PointCloudRendererView( glv::Rect( 360, 360 ), boost::shared_ptr< L3::PointCloud<double> >(), boost::shared_ptr<L3::SE3>() ) );
            //this->renderables.push_front( runtime_cloud_renderer_view.get() );
            updater->operator<<(  dynamic_cast<L3::Visualisers::Updateable*>(runtime_cloud_renderer_view.get() ) );

            point_cloud_maximise_controller.reset( new DoubleClickMaximiseToggle( runtime_cloud_renderer_view.get() ) );

            *ancillary_1 << *runtime_cloud_renderer_view;


            // Stand-alone scan renderer : Horizontal
            horizontal_scan_renderer.reset( new L3::Visualisers::HorizontalScanRenderer2DView( boost::shared_ptr< L3::ConstantTimeIterator< L3::LMS151 > >() , glv::Rect( 180,180 ) ) );
            updater->operator<<( horizontal_scan_renderer.get() );
            (*ancillary_1) << dynamic_cast<glv::View*>(horizontal_scan_renderer.get());
            window_controllers.push_back( boost::make_shared< DoubleClickMaximiseToggle >( dynamic_cast< glv::View* > (horizontal_scan_renderer.get() )) );

            // Stand-alone scan renderer : Vertical
            vertical_scan_renderer.reset( new L3::Visualisers::VerticalScanRenderer2DView( boost::shared_ptr< L3::ConstantTimeIterator< L3::LMS151 > >(), glv::Rect( 180,180 ) ) );
            updater->operator<<( vertical_scan_renderer.get() );
            window_controllers.push_back( boost::make_shared< DoubleClickMaximiseToggle >( dynamic_cast< glv::View* >(vertical_scan_renderer.get() ) ) );
            (*ancillary_1) << dynamic_cast<glv::View*>(vertical_scan_renderer.get());

            // Stand-alone pose renderer
            oracle_renderer.reset( new L3::Visualisers::DedicatedPoseRenderer( boost::shared_ptr<L3::PoseProvider>(), glv::Rect( 180,180 ), std::string("Estimate::INS" ) ) );
            updater->operator<<( oracle_renderer.get() );
            *ancillary_1 << *oracle_renderer;

            // Experience renderer
            experience_location.reset( new ExperienceLocationOverviewView( glv::Rect(180,180), boost::shared_ptr<L3::Experience>()  ) ); 
            window_controllers.push_back( boost::make_shared< DoubleClickMaximiseToggle >( experience_location.get() ) );
            *ancillary_1 << *experience_location;

            // Scan-matching scan renderer
            scan_matching_renderer.reset( new L3::Visualisers::ScanMatchingScanRenderer( glv::Rect( 180,180 ),boost::shared_ptr< L3::ScanMatching::Engine >() ) );
            window_controllers.push_back( boost::make_shared< DoubleClickMaximiseToggle >( dynamic_cast< glv::View* >(scan_matching_renderer.get() ) ) );
            *ancillary_1 << *scan_matching_renderer;

            // Dataset scaling factor
            scale_factor_label.reset( new glv::Label() );
            scale_factor.reset( new glv::Slider(glv::Rect(window.width()-155,window.height()-20,150, 10) ) );
            scale_factor->interval( 5, 1 );

            top << *scale_factor;
                        
            //time_renderer.reset( new TextRenderer<double>( runner->current_time ) );
            time_renderer.reset( new TextRenderer<double>() );
            time_renderer->pos(window.width()-155, window.height()-50 );

            top << *time_renderer;

            // Arrange
            dynamic_cast< glv::Table* >(ancillary_1.get())->arrange();

            ancillary_1->enable( glv::Property::DrawBorder );
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
            L3::Configuration::Begbroke begbroke;
            begbroke.loadDatum();


            // Remove it, if it is already in the composite list     
            composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>( map_view.get() ) );
            map_view = L3::Visualisers::LocaleRendererFactory::build( begbroke );
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
        
        
        }

        /*
         *  Estimator layout
         */
        EstimatorLayout::EstimatorLayout( glv::Window& win) : DatasetLayout(win)
        {

            pyramid_renderer.reset( new L3::Visualisers::HistogramPyramidRendererView(  glv::Rect( 150*3, 150 ), boost::shared_ptr< L3::HistogramPyramid<double> >(), 3 ) );

            for ( std::deque< boost::shared_ptr< HistogramDensityRenderer > >::iterator it = pyramid_renderer->renderers.begin();
                    it != pyramid_renderer->renderers.end();
                    it++ )
                updater->operator<<( it->get() );

            pyramid_renderer->pos( window.width() - (175+5), 0 );

            //top << *pyramid_renderer;
        
            //histogram_bounds_renderer.reset( new L3::Visualisers::HistogramBoundsRenderer( (*experience->experience_pyramid)[0]) );
            histogram_bounds_renderer.reset( new L3::Visualisers::HistogramBoundsRenderer( boost::shared_ptr<L3::Histogram<double> >() ) );
            histogram_bounds_renderer->depth = -2.0 ;
        }

        bool EstimatorLayout::load( L3::EstimatorRunner* runner, boost::shared_ptr<L3::Experience> experience )
        {
            /*
             *  Do parent
             */
            DatasetLayout::load( runner );

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
            histogram_voxel_renderer_experience_leaf.reset( new L3::Visualisers::HistogramVoxelRendererLeaf( (*experience->experience_pyramid)[0] ) ) ;
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(histogram_voxel_renderer_experience_leaf.get() ))); 

            //// Estimated pose
            //estimated_pose_renderer.reset( new L3::Visualisers::PoseRenderer( *runner->estimated ) );
            //this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(estimated_pose_renderer.get() ))); 

            // Predicted estimates
            composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>( algorithm_costs_renderer.get() ) );
            ////algorithm_costs_renderer.reset( new L3::Visualisers::AlgorithmCostRendererLeaf( dynamic_cast<L3::Estimator::IterativeDescent<double>* >( runner->algorithm ) ));
            //algorithm_costs_renderer.reset( new L3::Visualisers::AlgorithmCostRendererLeaf( boost::dynamic_pointer_cast< L3::Estimator::IterativeDescent<double> >( runner->algorithm ) ));
            //algorithm_costs_renderer->draw_bounds = true;
            //this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(algorithm_costs_renderer.get() ))); 

            /*
             *  Stand-alone plots
             */
            pyramid_renderer->loadPyramid( experience->experience_pyramid );

            /*
             *  Cost visualisation 
             */
            ////cost_renderer_view.reset( new L3::Visualisers::CostRendererView( *(runner->estimator->pose_estimates),  glv::Rect( win.width()-510,  win.height()-250, 200, 200 ) ) );
            ////this->renderables.push_front( cost_renderer_view.get() );

            ////dumper.reset( new DataDumper( runner->dumps ) );
            ////main_view->addGlobalInterface( glv::Event::KeyDown, dumper.get() );

            //// Truly dbg
            ////debug_renderer.reset( new L3::Visualisers::PointCloudRendererLeaf( runner->estimator->current_swathe ));
            ////this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(debug_renderer.get() ) ) );
            ////debug_renderer->color = glv::Color( 160, 32, 240 ); 

            ////debug_histogram_bounds_renderer.reset( new L3::Visualisers::HistogramBoundsRenderer( runner->estimator->current_histogram ) );
            ////this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(debug_histogram_bounds_renderer.get() ) ) );
            ////debug_histogram_bounds_renderer->depth = -12.0;
        }
    }
}
