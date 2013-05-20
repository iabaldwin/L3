#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include <boost/scoped_ptr.hpp>

#include <readline/readline.h>

#include "L3.h"
#include "Visualisers.h"
#include "VisualiserRunner.h"

#include "DatasetTools.h"

int main( int argc, char* argv[] )
{
    if ( argc != 2 ) 
    {
        std::cerr << "Usage: " << argv[0] << " <dataset>" << std::endl;
        exit(-1);
    }

    char* dataset_directory = argv[1];
 
    /*
     *  L3
     */
    //L3::Dataset dataset( dataset_directory );
    boost::shared_ptr< L3::Dataset > dataset( new L3::Dataset( dataset_directory ) );

    if( !( dataset->validate() && dataset->load() ) )
        exit(-1);
    
    /*
     *Configuration
     */
    L3::Configuration::Mission mission( *dataset );

    // Create runner
    boost::shared_ptr< L3::DatasetRunner > runner( new L3::DatasetRunner( dataset.get(), &mission) );

    runner->start();

    std::stringstream ss;
    ss.precision( 16 );

    L3::Visualisers::ExperienceBuilder builder( dataset, runner->current_time );

    /*
     *  Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::DatasetOverview");

    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
    
    L3::Visualisers::Composite composite;
    L3::Visualisers::CompositeController        composite_controller( &composite, composite.position );
    L3::Visualisers::DatasetOverviewView        view( glv::Rect(800,800), dataset, runner->provider  ); 

    L3::Visualisers::IteratorRenderer<L3::SE3>  iterator_renderer( runner->pose_iterator );

    top << view;

    top.addHandler( glv::Event::KeyDown, builder );

    // Run
    win.setGLV(top);
    glv::Application::run();

}
