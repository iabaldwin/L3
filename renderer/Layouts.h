#ifndef L3_VISUALISERS_LAYOUT_H
#define L3_VISUALISERS_LAYOUT_H

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "Visualisers.h"
#include "Plotters.h"

/*
 *Arbitrary text renderer
 */
namespace L3
{
namespace Visualisers
{

class Layout
{
    public:
        
        Layout( glv::Window& win ) : window(win)
        {
            // Create the main view
            main_view = new glv::View( glv::Rect(0,0, .6*win.width(),500));
            this->renderables.push_front( main_view );

            // Composite view holder
            composite.reset( new L3::Visualisers::Composite( glv::Rect(.6*win.width(), 500 )) );
            
            // 3D grid 
            grid.reset( new L3::Visualisers::Grid() );
           
            // Basic controller
            controller.reset( new L3::Visualisers::BasicPanController() );
        
            composite->addController( &*controller );

            // Accumulate views
            (*main_view) << ( *composite << *grid );
       
            // Add synched updater
            updater.reset( new Updater() );
            this->renderables.push_front( updater.get() );
        }
        
        virtual ~Layout()
        {
        }

        void run( glv::GLV& top )
        {
            top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
           
            // Add renderables provided by children
            for( std::list< glv::View* >::iterator it = renderables.begin(); it != renderables.end(); it++ )
                top << *it;

            window.setGLV(top);

            glv::Application::run();
        }

        void addRotationalVelocityPlot( L3::ConstantTimeIterator< L3::LHLV >* LHLV_iterator )
        {
            /*
             *  Linear Velocity
             */
            
            // Add plotter
            boost::shared_ptr< VelocityPlotter > plotter( new L3::Visualisers::RotationalVelocityPlotter( LHLV_iterator ) );
            plotter->stroke( 2.0 );
            plotter->drawUnderGrid(true);

            plotters.push_front( plotter );

            // Add plot region
            boost::shared_ptr< glv::Plot > plot_region( new glv::Plot( glv::Rect( 0, 650+5, .6*window.width(), 150-5), *plotter ) );

            plot_region->range( 0, 1000, 0 );
            plot_region->range( -1, 1, 1 );

            plot_region->numbering(true);
            plot_region->showNumbering(true);
        
            plots.push_front( plot_region );
            
            // Add rendererable
            this->renderables.push_front( plot_region.get() );

            updater->operator<<( dynamic_cast<Updateable*>(plotter.get()) );
        }


        void addLinearVelocityPlot( L3::ConstantTimeIterator< L3::LHLV >* LHLV_iterator )
        {
            /*
             *  Linear Velocity
             */
            
            // Add plotter
            boost::shared_ptr< VelocityPlotter > plotter( new L3::Visualisers::LinearVelocityPlotter( LHLV_iterator ) );
            plotter->stroke( 2.0 );
            plotter->drawUnderGrid(true);

            plotters.push_front( plotter );

            // Add plot region
            boost::shared_ptr< glv::Plot > plot_region( new glv::Plot( glv::Rect( 0, 500+5, .6*window.width(), 150-5), *plotter ) );

            plot_region->range( 0, 1000, 0 );
            plot_region->range( -1, 10 , 1 );

            plot_region->numbering(true);
            plot_region->showNumbering(true);
        
            plots.push_front( plot_region );
            
            // Add rendererable
            this->renderables.push_front( plot_region.get() );

            updater->operator<<( dynamic_cast<Updateable*>(plotter.get()) );
        }

    protected:

        glv::View*   main_view;
        glv::Window& window; 

        std::list< glv::View* > renderables;
        
        boost::shared_ptr< Updater >                    updater;
        boost::shared_ptr<L3::Visualisers::Grid>        grid;
        boost::shared_ptr<L3::Visualisers::Composite>   composite;
        boost::shared_ptr<L3::Visualisers::Controller>  controller;
        
        std::list< boost::shared_ptr< glv::Plot > >         plots;
        std::list< boost::shared_ptr< VelocityPlotter > >   plotters;

};


/*
 *  Dataset 
 */
class DatasetLayout : public Layout
{
    public:

        DatasetLayout( glv::Window& win, L3::Dataset* dataset ) : Layout(win), dataset(dataset)
        {
            // Start the dataset runner
            runner.reset( new L3::DatasetRunner( dataset ) );
            runner->start( dataset->start_time );

            addLinearVelocityPlot( runner->LHLV_iterator.get() );
            addRotationalVelocityPlot( runner->LHLV_iterator.get() );
            
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

        EstimatorLayout( glv::Window& win, L3::EstimatorRunner* runner, boost::shared_ptr<L3::Experience> experience, L3::PointCloud<double>* run_time_swathe ) 
            : Layout(win), runner(runner), experience(experience)
        {
            // Histogram voxels
            histogram_pixel_renderer_experience.reset( new L3::Visualisers::HistogramPixelRenderer( glv::Rect(500, 300 ), experience->experience_histogram ) ) ;
            histogram_pixel_renderer_experience->pos(win.width()-(500+10),10);
            this->renderables.push_front( histogram_pixel_renderer_experience.get() );
            updater->operator<<( dynamic_cast<Updateable*>(histogram_pixel_renderer_experience.get()) );
            
            // Histogram Bounds
            histogram_bounds_renderer.reset( new L3::Visualisers::HistogramBoundsRenderer( experience->experience_histogram ) );
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(histogram_bounds_renderer.get() ) ) );

            // Swathe Bounds
            point_cloud_bounds_renderer.reset( new L3::Visualisers::PointCloudBoundsRenderer ( run_time_swathe ) );
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(point_cloud_bounds_renderer.get() ) ) );

            // Swathe Cloud
            runtime_cloud_renderer.reset( new L3::Visualisers::PointCloudRenderer( run_time_swathe ));
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(runtime_cloud_renderer.get() ) ) );

            // Velocity plots
            addLinearVelocityPlot( runner->windower->constant_time_iterator );
            addRotationalVelocityPlot( runner->windower->constant_time_iterator );

            // Current pose estimate
            pose_renderer.reset( new L3::Visualisers::PoseRenderer( *runner->current ) );
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(pose_renderer.get() ))); 
        
            // Predicted estimates
            predictor_renderer.reset( new L3::Visualisers::PredictorRenderer( runner->estimator->pose_estimates ) ); 
            this->composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(predictor_renderer.get() ))); 
        
            // Stand-alone pose renderer
            oracle_rendere.reset( new L3::Visualisers::DedicatedPoseRenderer( runner->provider, glv::Rect( 240,240 ) ) );
            oracle_rendere->pos( win.width()-(240+10), 325 );
            this->renderables.push_front( oracle_rendere.get() );
            updater->operator<<( oracle_rendere.get() );
       

            // Stand-alone pose renderer
            predicted_pose_renderer.reset( new L3::Visualisers::DedicatedPoseRenderer( runner->provider, glv::Rect( 240,240 ) ) );
            predicted_pose_renderer->pos( win.width()-(240*2+20), 325 );
            this->renderables.push_front( predicted_pose_renderer.get() );
            updater->operator<<( predicted_pose_renderer.get() );
       

        }
    
        L3::EstimatorRunner* runner;
      
        boost::shared_ptr< L3::Visualisers::PoseRenderer >  pose_renderer;

        boost::shared_ptr< L3::Experience> experience ;
        
        boost::shared_ptr< L3::Visualisers::DedicatedPoseRenderer>      oracle_rendere;
        boost::shared_ptr< L3::Visualisers::DedicatedPoseRenderer>      predicted_pose_renderer;
        
        boost::shared_ptr< L3::Visualisers::PredictorRenderer >         predictor_renderer;
        boost::shared_ptr< L3::Visualisers::PointCloudRenderer >        runtime_cloud_renderer; 
        boost::shared_ptr< L3::Visualisers::HistogramBoundsRenderer >   histogram_bounds_renderer;
        boost::shared_ptr< L3::Visualisers::PointCloudBoundsRenderer >  point_cloud_bounds_renderer;
        boost::shared_ptr< L3::Visualisers::HistogramPixelRenderer >    histogram_pixel_renderer_experience;

};


} // Visualisers
} // L3


#endif

