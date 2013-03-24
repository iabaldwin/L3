#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "Components.h"

template <typename T>
struct Transformer : L3::Visualisers::Leaf
{

    Transformer( L3::PointCloud<T>* point_cloud ) : cloud( point_cloud )
    {
        done = false;
    }

    L3::PointCloud<T>* cloud;

    bool done;
    void onDraw3D( glv::GLV& g )
    {
        L3::SE3 update( 0, 0, -10.0, 0, 0, 0 );
  
        transform( cloud, &update );

        if (!done)
        {
            delete [] cloud->points;
            done = true;
        }
    }

};


int main (int argc, char ** argv)
{
    /*
     *L3
     */
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );
    if( !( dataset.validate() && dataset.load() ) )
        throw std::exception();
    
    // Constant time iterator over poses
    L3::ConstantTimeIterator< L3::SE3 >  pose_iterator( dataset.pose_reader, 10.0 );
    
    // Constant time iterator over LIDAR
    L3::ConstantTimeIterator< L3::LMS151 > LIDAR_iterator( dataset.LIDAR_readers.front(), 10.0 );
    
    double time = dataset.start_time;

    L3::ConstantTimePoseWindower pose_windower( &pose_iterator );
    
    L3::SwatheBuilder swathe_builder( &pose_windower, &LIDAR_iterator );

    for( int i=0;i<100;i++ )
        swathe_builder.update( time += .5 );

    L3::PointCloud<double> cloud;

    // Projector  
    L3::SE3 calibration = L3::SE3( 0, 0, 0, -1.57, 0, 0 ); 
    std::auto_ptr<L3::Projector<double> > projector( new L3::Projector<double>( &calibration, &cloud ) );

    // Do Projection
    projector->project( swathe_builder.swathe );

    std::cout << cloud.num_points << std::endl;

    // Sample
    L3::PointCloud<double> sampled_cloud = L3::samplePointCloud( &cloud, 10000 );

    /*
     *Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::PointCloud");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
    
    L3::Visualisers::Composite          composite_view;
    L3::Visualisers::BasicPanController controller;
    L3::Visualisers::Grid               grid;
    
    composite_view.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) );
   
    // Transform tester
    Transformer<double> transformer( &sampled_cloud );
    
    // Point cloud renderer
    L3::Visualisers::CloudRenderer<double> cloud_renderer( &sampled_cloud );

    composite_view << cloud_renderer << transformer << grid;
    top << composite_view;

    win.setGLV(top);
    glv::Application::run();
}


