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

    L3::Tools::Timer t;

    t.begin();
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );
    if( !( dataset.validate() && dataset.load() ) )
        throw std::exception();
    
    std::cout << "Dataset loaded in " << t.end() << "s" << std::endl; 

    std::string LIDAR_name = dataset.LIDAR_names[0];
    L3::ConstantTimeIterator* iterator = new L3::ConstantTimeIterator( &dataset, LIDAR_name, 40.0 );

    L3::Utils::localisePoseChainToMean( dataset.poses );

    // Projector  
    std::auto_ptr<L3::Projector<double> > projector( new L3::Projector<double>() );

    // Get swathe
    SWATHE* swathe = iterator->getSwathe();

    // Do Projection
    L3::PointCloudXYZ<double> cloud = projector->project( iterator->getSwathe() );
    
    // Sample
    L3::PointCloudXYZ<double> sampled_cloud = L3::samplePointCloud( cloud, 10000 );

    /*
     *Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::PointCloud");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
    
    glv::Grid grid(glv::Rect(0,0));
    grid.range(1);            // set plot region
    grid.major(1);            // set major tick mark placement
    grid.minor(2);            // number of divisions per major ticks
    grid.equalizeAxes(true);
    grid.stretch(1,.2);

    double d = 800;
    glv::Plot v1__( glv::Rect( 0,0*d/8, d, d/8), *new glv::PlotFunction1D(glv::Color(0.5,0,0)));

    // Point cloud renderer
    //L3::Visualisers::CloudRenderer<double> cloud_renderer( &cloud );
    L3::Visualisers::CloudRenderer<double> cloud_renderer( &sampled_cloud );
    
    // Pose chain renderer
    std::vector<L3::Pose*> poses = L3::Utils::posesFromSwathe( iterator->getSwathe() );
    L3::Visualisers::PoseChainRenderer pose_chain_renderer( poses );  
    
    L3::Visualisers::Composite composite_view;
    composite_view << pose_chain_renderer << cloud_renderer;
    top << composite_view;

    win.setGLV(top);
    glv::Application::run();
}


