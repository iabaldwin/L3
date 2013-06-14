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
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" );
    if( !( dataset.validate() && dataset.load() ) )
        throw std::exception();

    L3::Configuration::Mission mission( dataset );

    // Constant time iterator over LHLV
    L3::ConstantTimeIterator< L3::LHLV > iterator( dataset.LHLV_reader );
    
    // Constant time iterator over LIDAR
    L3::ConstantTimeIterator< L3::LMS151 > LIDAR_iterator( dataset.LIDAR_readers[ mission.declined ] );
    
    L3::ConstantTimeWindower<L3::LHLV> pose_windower( &iterator ); 

    /*
     *Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::PointCloud");

    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
    
    L3::Visualisers::Composite              composite;
    L3::Visualisers::CompositeController    controller( &composite, composite.position );
    L3::Visualisers::Grid                   grid;
    L3::Visualisers::PoseWindowerRenderer   pose_renderer( &pose_windower ); 

    top << (composite << pose_renderer << grid );

    composite.stretch(1,1);

    win.setGLV(top);
    win.fit(); 
    glv::Application::run();
}


