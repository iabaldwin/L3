#include "Layouts.h"

namespace L3
{
    namespace Visualisers
    {

        EstimatorLayout::EstimatorLayout( glv::Window& win, L3::EstimatorRunner* runner, boost::shared_ptr<L3::Experience> experience, boost::shared_ptr< L3::PointCloud<double> > run_time_swathe ) 
            : Layout(win), runner(runner), experience(experience)
        {

            /*
             *  Composite Leafs
             */
            // Histogram Bounds
            histogram_bounds_renderer.reset( new L3::Visualisers::HistogramBoundsRenderer( experience->experience_histogram ) );
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(histogram_bounds_renderer.get() ) ) );

            // Histogram voxel
            histogram_voxel_renderer_experience_leaf.reset( new L3::Visualisers::HistogramVoxelRendererLeaf( experience->experience_histogram ) ) ;
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(histogram_voxel_renderer_experience_leaf.get() ))); 

            // Swathe Bounds
            point_cloud_bounds_renderer.reset( new L3::Visualisers::PointCloudBoundsRenderer ( run_time_swathe ) );
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(point_cloud_bounds_renderer.get() ) ) );

            // Swathe Cloud
            runtime_cloud_renderer_leaf.reset( new L3::Visualisers::PointCloudRendererLeaf( run_time_swathe ));
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(runtime_cloud_renderer_leaf.get() ) ) );

            // Current pose estimate
            pose_renderer.reset( new L3::Visualisers::PoseRenderer( *runner->current ) );
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(pose_renderer.get() ))); 

            // Predicted estimates
            predictor_renderer.reset( new L3::Visualisers::PredictorRenderer( runner->estimator->pose_estimates ) ); 
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(predictor_renderer.get() ))); 

            /*
             *  Stand-alone plots
             */
            // Velocity plots
            addLinearVelocityPlot( runner->windower->constant_time_iterator );
            addRotationalVelocityPlot( runner->windower->constant_time_iterator );

            // Histogram voxel
            //  This, is hard to get right it seems
            //histogram_pixel_renderer_experience_view.reset( new L3::Visualisers::HistogramVoxelRendererView( glv::Rect( win.width()-(500+10),10, 500, 300 ), experience->experience_histogram ) ) ;
            //this->renderables.push_front( histogram_pixel_renderer_experience_view.get() );
            //updater->operator<<( dynamic_cast<Updateable*>(histogram_pixel_renderer_experience_view.get()) );

            //boost::shared_ptr< glv::View > histogram_label( new glv::Label("Experience histogram") );
            //histogram_label->pos( 1050,315 );
            //this->labels.push_front( histogram_label );
            //this->renderables.push_front( histogram_label.get() );

            // Stand-alone pose renderer
            oracle_renderer.reset( new L3::Visualisers::DedicatedPoseRenderer( runner->provider, glv::Rect( 150,150 ) ) );
            oracle_renderer->pos( win.width()-(150+10), 335 );
            this->renderables.push_front( oracle_renderer.get() );
            updater->operator<<( oracle_renderer.get() );

            boost::shared_ptr< glv::View > oracle_label( new glv::Label("Estimate::INS") );
            oracle_label->pos( win.width()-(150+10), 335+160 );
            this->labels.push_front( oracle_label );
            this->renderables.push_front( oracle_label.get() );

            // Stand-alone pose renderer
            //predicted_pose_renderer.reset( new L3::Visualisers::DedicatedPoseRenderer( runner->provider, glv::Rect( 150,150 ) ) );
            //predicted_pose_renderer->pos( win.width()-(150*2+30), 335 );
            //this->renderables.push_front( predicted_pose_renderer.get() );
            //updater->operator<<( predicted_pose_renderer.get() );

            //// ::Label
            //boost::shared_ptr< glv::View > predicted_pose_label( new glv::Label("Estimate::L3") );
            //predicted_pose_label->pos( win.width()-(150*2+30), 335+250 );
            //this->labels.push_front( predicted_pose_label );
            //this->renderables.push_front( predicted_pose_label.get() );

            // Stand-alone scan renderer :: Horizontal
            horizontal_scan_renderer.reset( new L3::Visualisers::HorizontalScanRenderer2D( runner->horizontal_LIDAR, glv::Rect( 150,150 ) ) );
            horizontal_scan_renderer->pos( win.width()-(150*3+60), 335 );
            this->renderables.push_front( horizontal_scan_renderer.get() );
            updater->operator<<( horizontal_scan_renderer.get() );

            boost::shared_ptr< glv::View > horizontal_scan_renderer_label( new glv::Label("LMS151::Horizontal") );
            horizontal_scan_renderer_label->pos( win.width()-(150*3+60), 335+160 );
            this->labels.push_front( horizontal_scan_renderer_label );
            this->renderables.push_front( horizontal_scan_renderer_label.get() );

            // Stand-alone scan renderer : Vertical
            vertical_scan_renderer.reset( new L3::Visualisers::VerticalScanRenderer2D( runner->vertical_LIDAR, glv::Rect( 150,150 ) ) );
            vertical_scan_renderer->pos( win.width()-(150*2+30), 335 );
            this->renderables.push_front( vertical_scan_renderer.get() );
            updater->operator<<( vertical_scan_renderer.get() );

            boost::shared_ptr< glv::View > vertical_scan_renderer_label( new glv::Label("LMS151::Vertical") );
            vertical_scan_renderer_label->pos( win.width()-(150*2+30), 335+160 );
            this->labels.push_front( vertical_scan_renderer_label );
            this->renderables.push_front( vertical_scan_renderer_label.get() );
            
            // Swathe Cloud
            runtime_cloud_renderer_view.reset( new L3::Visualisers::PointCloudRendererView( glv::Rect( win.width()-(500+10),10, 500, 300 ), run_time_swathe, runner->current ));
            this->renderables.push_front( runtime_cloud_renderer_view.get() );
            updater->operator<<( runtime_cloud_renderer_view.get() );
        
            // Cost visualisation 
            cost_renderer_view.reset( new L3::Visualisers::CostRendererView( *(runner->estimator->pose_estimates),  glv::Rect( win.width()-510,  win.height()-250, 200, 200 ) ) );
            this->renderables.push_front( cost_renderer_view.get() );

            
        }
    }
}
