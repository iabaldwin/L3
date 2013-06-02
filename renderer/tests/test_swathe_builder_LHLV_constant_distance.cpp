#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "VisualiserRunner.h"

#include <boost/scoped_ptr.hpp>

#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>

int main (int argc, char ** argv)
{
    /*
     *  L3
     */
    boost::scoped_ptr< L3::Dataset > dataset( new L3::Dataset( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" ) );
    if( !( dataset->validate() && dataset->load() ) )
        throw std::exception();
    
    L3::Configuration::Mission mission( *dataset );

    // Constant time iterator over LHLV data
    L3::ConstantTimeIterator< L3::LHLV >   LHLV_iterator( dataset->LHLV_reader );

    // Constant time iterator over LIDAR
    L3::ConstantTimeIterator< L3::LMS151 > LIDAR_iterator( dataset->LIDAR_readers[ mission.declined ] );

    L3::ConstantDistanceWindower pose_windower( &LHLV_iterator, 60 );

    L3::RawSwatheBuilder swathe_builder( &pose_windower, &LIDAR_iterator );

    /*
     *  Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation");

    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    L3::Visualisers::Composite              composite;
    L3::Visualisers::CompositeController    controller( &composite, composite.position );
    L3::Visualisers::Grid                   grid;
    L3::Visualisers::SwatheRenderer         swathe_renderer( &swathe_builder ); 
    L3::Visualisers::PoseWindowerRenderer   pose_renderer( &pose_windower ); 

    composite.stretch(1,1);

    L3::Visualisers::VisualiserRunner runner( dataset->start_time );

    runner << &swathe_builder << &pose_windower << &LIDAR_iterator << &LHLV_iterator;

    top << (composite << swathe_renderer << pose_renderer << grid  << runner);

    win.setGLV(top);

    glv::Application::run();
}


