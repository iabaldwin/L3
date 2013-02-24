#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"

int main (int argc, char ** argv)
{
    /*
     *L3
     */
    // Pose sequence

    //L3::Tools::Timer t;

    //t.begin();
    //L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );
    //if( !( dataset.validate() && dataset.load() ) )
        //throw std::exception();
    
    //// Constant time iterator over poses
    //L3::ConstantTimeIterator< L3::Pose >  pose_iterator( dataset.pose_reader, 10.0 );
    //// Constant time iterator over LIDAR
    //L3::ConstantTimeIterator< L3::LIDAR > LIDAR_iterator( dataset.LIDAR_readers.front(), 10.0 );
    
    //double time = dataset.start_time;

    //L3::SwatheBuilder swathe_builder( &pose_iterator, &LIDAR_iterator );

    //while( swathe_builder.duration() < 10.0 )
        //swathe_builder.update( time += .1 );

    //// Projector  
    //std::auto_ptr<L3::Projector<double> > projector( new L3::Projector<double>() );

    //// Do Projection
    //L3::PointCloudXYZ<double> cloud = projector->project( swathe_builder.swathe );
    
    //// Sample
    //L3::PointCloudXYZ<double> sampled_cloud = L3::samplePointCloud( cloud, 10000 );

    /*
     *Visualisation
     */
    //glv::GLV top;
    //glv::Window win(1400, 800, "Visualisation::PointCloud");

    //// Colors
    //top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
    
    //glv::Grid grid(glv::Rect(0,0));
    //grid.range(1);            // set plot region
    //grid.major(1);            // set major tick mark placement
    //grid.minor(2);            // number of divisions per major ticks
    //grid.equalizeAxes(true);
    //grid.stretch(1,.2);

    //double d = 800;
    //glv::Plot v1__( glv::Rect( 0,0*d/8, d, d/8), *new glv::PlotFunction1D(glv::Color(0.5,0,0)));

    //// Point cloud renderer
    //L3::Visualisers::CloudRenderer<double> cloud_renderer( &sampled_cloud );
    //L3::Visualisers::Composite composite_view;
  
    ////std::ofstream cloud_output( "cloud.dat" );
    ////cloud_output << sampled_cloud;
    ////cloud_output.close();

    //composite_view << cloud_renderer;
    //top << composite_view;

    //win.setGLV(top);
    //glv::Application::run();
}


