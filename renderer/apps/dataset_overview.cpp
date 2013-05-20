#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "VisualiserRunner.h"

int main (int argc, char ** argv)
{

    if ( argc != 2 ) 
    {
        std::cerr << "Usage: " << argv[0] << " <dataset>" << std::endl;
        exit(-1);
    }

    char* dataset_target = argv[1];
 

    /*
     *L3
     */
    boost::shared_ptr< L3::Experience > experience;

    boost::shared_ptr< L3::Dataset > dataset;

    try
    {
        /*
         *  Experience
         */
        L3::Dataset experience_dataset( dataset_target );
        L3::ExperienceLoader experience_loader( experience_dataset );
        experience = experience_loader.experience;
  
        /*
         *  Dataset
         */
        dataset.reset( new L3::Dataset( dataset_target ) );
    
    }
    catch( std::exception e )
    {
        std::cout << e.what() << std::endl;
        return -1;
    }

    // Configuration
    L3::Configuration::Mission* mission = new L3::Configuration::Mission( *dataset ) ;

    // Create runner
    boost::shared_ptr< L3::DatasetRunner > runner( new L3::DatasetRunner( dataset.get(), mission ) );
    runner->start();
    
    /*
     *  Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::DatasetOverview");

    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
    
    L3::Visualisers::ExperienceLocationOverviewView     experience_view( glv::Rect((2.0/3.0)*win.width(),(2.0/3.0)*win.height()), experience, runner->provider );

    //experience_view.stretch(1,1);

    top << experience_view;

    // Run
    win.setGLV(top);
    glv::Application::run();

}
