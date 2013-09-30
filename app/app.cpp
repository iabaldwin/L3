#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "L3Vis.h"

int main( int argc, char* argv[] )
{
    if ( argc != 2 ) 
    {
        std::cerr << "Usage: " << argv[0] << " <dataset>" << std::endl;
        return(-1);
    }

    char* dataset_directory = argv[1];
 
    /*
     *  L3
     */
    L3::Dataset* dataset;
       
    try
    {
        dataset = new L3::Dataset( dataset_directory );
    }
    catch(...)
    {
        std::cerr << "No such dataset <" << dataset_directory  << ">" << std::endl;
        return(-1);
    }

    if( !dataset->validate()  )
    {
        std::cerr << "Failed to validate <" << dataset_directory << ">" << std::endl;
        return(-1);
    }

    if( !dataset->load() )
    {
        std::cerr << "Failed to load <" << dataset_directory << ">" << std::endl;
        return(-1);
    }

    // Configuration
    L3::Configuration::Mission* mission;
    try
    {
        mission = new L3::Configuration::Mission( *dataset ) ;
    }
    catch( std::exception& e )
    {
        std::cerr << "Unable to load a configuration file for " << *dataset << std::endl;
        return( -1  );
    }

    // Experience - Fixed
    L3::Dataset experience_dataset( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" );
    L3::ExperienceLoader experience_loader( experience_dataset );
    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;

    // Estimator
    L3::Estimator::CostFunction<double>* cost_function = new L3::Estimator::MICostFunction<double>();

    // Create runner
    boost::shared_ptr< L3::EstimatorRunner > runner( new L3::EstimatorRunner( dataset, mission, experience.get() ) );
   
    // Build algorithm
    boost::shared_ptr< L3::Estimator::Algorithm<double> > algo( new L3::Estimator::UKF<double>( boost::shared_ptr< L3::Estimator::CostFunction<double> >(cost_function), experience->experience_pyramid, runner->ics_velocity_provider ) );
    
    runner->setAlgorithm( algo );
    runner->start();

    // Fold into container
    boost::shared_ptr< L3::Container > container( new L3::Container() );

    container->dataset    = boost::shared_ptr<L3::Dataset>( dataset );
    container->mission    = boost::shared_ptr< L3::Configuration::Mission >( mission );
    container->experience = experience;
    container->runner     = runner;

    glv::Window win(1400, 800, "Visualisation::Estimation");

    L3::Visualisers::EstimatorLayout layout( win );
    
    L3::Interface* command_interface = new L3::CommandInterface( &layout, container );
    L3::Interface* lua_interface = new L3::LuaInterface();

    (*static_cast< L3::Visualisers::GLVInterface* >( layout.scripting_interface.get() ) ) << command_interface << lua_interface;

    layout.load( runner.get(), experience );

    layout.run();

}
