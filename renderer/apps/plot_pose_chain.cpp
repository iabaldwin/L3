#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "Controller.h"

int main (int argc, char ** argv)
{
    L3::SE3 calibration = L3::SE3::ZERO();
    
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-08-09-36-42-WOODSTOCK-SLOW/" );
    dataset.validate();


    try
    {
        calibration = L3::Utils::loadCalibration( "dataset/" + dataset.name()  + ".config", "LMS1xx_10420002" );
    }
    catch( L3::calibration_failure )
    {
        std::cout << "No/erroneous calibration file for " <<  dataset.name() << std::endl;
        throw std::exception(); 
    }

    std::cout << calibration << std::endl;
    
    std::auto_ptr<L3::IO::BinaryReader< L3::SE3 > > INS_reader;
    std::auto_ptr<L3::IO::BinaryReader< L3::LMS151 > > LIDAR_reader;
        
    std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > poses;
    std::vector< std::pair< double, boost::shared_ptr<L3::LMS151> > > LIDAR_data;


    // Load the poses
    INS_reader.reset( new L3::IO::BinaryReader<L3::SE3>() );
    INS_reader->open( dataset.path() + "/OxTS.ins" );
    INS_reader->read();
    INS_reader->extract( poses );

    // Load the lidar data 
    LIDAR_reader.reset( new L3::IO::BinaryReader<L3::LMS151>() );
    LIDAR_reader->open( dataset.path() + "/LMS1xx_10420002_192.168.0.50.lidar" );
    LIDAR_reader->read();
    LIDAR_reader->extract( LIDAR_data );

    // Match
    std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > matched;
    L3::Utils::matcher< L3::SE3, L3::LMS151 > m( &poses, &matched );
    m = std::for_each( LIDAR_data.begin(), LIDAR_data.end(),  m );

    // Thread into swathe
    L3::Utils::threader<L3::SE3, L3::LMS151>  t;

    SWATHE s;

    std::transform( matched.begin(), 
            matched.end(), 
            LIDAR_data.begin(), 
            std::back_inserter( s ),
            t );


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
    L3::Visualisers::PoseChainRenderer  pose_chain_renderer( &poses );
    L3::Visualisers::Composite          composite_view;
    L3::Visualisers::Controller         controller;

    composite_view.current_time = dataset.start_time;
    composite_view.sf = 2.0;

    top << (composite_view << pose_chain_renderer);

    //top.addHandler( glv::Event::KeyDown, controller );
    //top.addHandler( glv::Event::KeyRepeat, controller );
    
    win.setGLV(top);
    glv::Application::run();

}
