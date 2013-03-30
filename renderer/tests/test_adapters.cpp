#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Adapters.h"

int main (int argc, char ** argv)
{
    /*
     *L3
     */
    // Pose sequence
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );
    if ( !(dataset.validate() && dataset.load() ))
        throw std::exception();
    
    std::string LIDAR_name = dataset.LIDAR_names[0];
    L3::ConstantTimeIterator< L3::SE3 > iterator( dataset.pose_reader );

    // Localise pose chain, for rendering
    //L3::Utils::localisePoseChainToMean( dataset.poses );

    // Projector 
    L3::SE3 calibration( 0, 0, 0, 0, 0, 0 );
    L3::PointCloud<double>* point_cloud = new L3::PointCloud<double>();
    std::auto_ptr<L3::Projector<double> > projector( new L3::Projector<double>( &calibration, point_cloud ) );

    // Do Projection
    //L3::PointCloudXYZ<double> cloud = projector->project( iterator->getSwathe() );

    //glv::Data data = L3::Visualisers::Adapter::Adapt<L3::LHLV>( dataset.LHLV_data );

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

    //double d = 1200;
    //glv::Plot v1__( glv::Rect( 0, d/8, d, d/8), *new glv::PlotFunction1D(glv::Color(0.5,0,0)));

    //v1__.data() =  data.slice(1);

    //// Point cloud renderer
    ////L3::Visualisers::CloudRenderer<double> cloud_renderer( &cloud );
    //// Pose chain renderer
    ////std::vector<L3::Pose*> poses = L3::Utils::posesFromSwathe( iterator->getSwathe() );
    ////L3::Visualisers::PoseChainRenderer pose_chain_renderer( poses );  
    
    ////L3::Visualisers::Composite composite_view;
    ////composite_view << pose_chain_renderer << cloud_renderer;
    ////top << composite_view;
    
    //top << v1__;

    //win.setGLV(top);
    //glv::Application::run();
}


