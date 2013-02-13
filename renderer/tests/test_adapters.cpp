#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "Datatypes.h"
#include "Dataset.h"
#include "Iterator.h"
#include "Projector.h"
#include "Utils.h"
#include "Reader.h"
#include "Writer.h"
#include "Visualisers.h"
#include "DataAdapters.h"


int main (int argc, char ** argv)
{
    /*
     *L3
     */
    // Pose sequence
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );
    assert( dataset.validate() && dataset.load() );
    
    std::string LIDAR_name = dataset.LIDAR_names[0];
    L3::ConstantTimeIterator* iterator = new L3::ConstantTimeIterator( &dataset, LIDAR_name, 60.0 );

    L3::Utils::localisePoseChainToMean( dataset.poses );

    // Projector  
    std::auto_ptr<L3::Projector<double> > projector( new L3::Projector<double>() );

    // Get swathe
    SWATHE* swathe = iterator->getSwathe();

    // Do Projection
    L3::PointCloudXYZ<double> cloud = projector->project( iterator->getSwathe() );

    std::ofstream cloud_writer( "cloud.dat" );
    cloud_writer << cloud;
    cloud_writer.close();

    std::ofstream pose_writer( "poses.dat" );
    std::vector< L3::Pose* > dbg_poses = L3::Utils::posesFromSwathe( iterator->getSwathe() );
    std::vector< L3::Pose* >::iterator it = dbg_poses.begin();
    while( it != dbg_poses.end() )
    {

        pose_writer << *(*it) << std::endl;
        it++;
    }
    pose_writer.close();

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
    //L3::Visualisers::CloudRenderer<double> cloud_renderer( &cloud );
    //// Pose chain renderer
    //std::vector<L3::Pose*> poses = L3::Utils::posesFromSwathe( iterator->getSwathe() );
    //L3::Visualisers::PoseChainRenderer pose_chain_renderer( poses );  
    
    //L3::Visualisers::Composite composite_view;
    //composite_view << pose_chain_renderer << cloud_renderer;
    //top << composite_view;

    //win.setGLV(top);
    //glv::Application::run();
}


