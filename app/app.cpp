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
    L3::Dataset* dataset{nullptr};
       
    try
    {
        dataset = new L3::Dataset( dataset_directory );
    }
    catch(...)
    {
        LOG(ERROR) << "No such dataset <" << dataset_directory  << ">";
        return(EXIT_FAILURE);
    }

    if( not dataset->validate()  )
    {
        LOG(ERROR) << "Failed to validate <" << dataset_directory << ">";
        return(EXIT_FAILURE);
    }

    if( !dataset->load() )
    {
        LOG(ERROR) << "Failed to load <" << dataset_directory << ">";
        return(-1);
    }

    // Configuration
    L3::Configuration::Mission* mission{nullptr};
    try
    {
        mission = new L3::Configuration::Mission( *dataset ) ;
    }
    catch( std::exception& e )
    {
        LOG(ERROR) << "Unable to load/validate configuration file for " << dataset->name() << " <" << e.what() << ">";
        return(EXIT_FAILURE);
    }

    // Experience - Fixed
#ifndef NDEBUG
    LOG(INFO) << "Loading fixed experience....";
#endif

    char * pPath;
    pPath = getenv ("L3");
    if (pPath==NULL) {
      LOG(ERROR) << "Failed to find L3 root (export L3=...)";
      exit(EXIT_FAILURE);
    }

    constexpr char default_dataset[] = "2012-04-16-20-05-30NightWoodstock1";

    L3::Dataset experience_dataset( std::string{pPath} + "/" +  default_dataset);
    L3::ExperienceLoader experience_loader( experience_dataset );
    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;

    // Estimator
    L3::Estimator::CostFunction<double>* cost_function = new L3::Estimator::MICostFunction<double>();
    CHECK_NOTNULL(cost_function);

    // Create runner
    boost::shared_ptr< L3::EstimatorRunner > runner( new L3::EstimatorRunner( dataset, mission, experience.get() ) );
    CHECK_NOTNULL(runner);

    // Build algorithm
    boost::shared_ptr< L3::Estimator::Algorithm<double> > algo( new L3::Estimator::UKF<double>( boost::shared_ptr< L3::Estimator::CostFunction<double> >(cost_function), experience->experience_pyramid, runner->ics_velocity_provider ) );
    CHECK_NOTNULL(algo);

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
    CHECK_NOTNULL(command_interface);
    L3::Interface* lua_interface = new L3::LuaInterface();
    CHECK_NOTNULL(lua_interface);

    (*static_cast< L3::Visualisers::GLVInterface* >( layout.scripting_interface.get() ) ) << command_interface << lua_interface;

    layout.load( runner.get(), experience );

    layout.run();
}
