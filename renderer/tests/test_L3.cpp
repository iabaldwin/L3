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
        exit(-1);
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
        exit(-1);
    }

    if( !( dataset->validate() && dataset->load() ) )
        exit(-1);

    // Configuration
    L3::Configuration::Mission* mission = new L3::Configuration::Mission( *dataset ) ;

    // Experience
    //L3::Dataset experience_dataset( "/Users/ian/code/datasets/2012-02-08-09-36-42-WOODSTOCK-SLOW/" );
    L3::Dataset experience_dataset( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" );
    L3::ExperienceLoader experience_loader( experience_dataset );
    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;

    // Estimator
    //L3::Estimator::CostFunction<double>* cost_function = new L3::Estimator::KLCostFunction<double>();
    L3::Estimator::CostFunction<double>* cost_function = new L3::Estimator::MICostFunction<double>();
    //L3::Estimator::CostFunction<double>* cost_function = new L3::Estimator::NMICostFunction<double>();
    //L3::Estimator::CostFunction<double>* cost_function = new L3::Estimator::RenyiMICostFunction<double>();
    //L3::Estimator::CostFunction<double>* cost_function = new L3::Estimator::SSDCostFunction<double>();
    
    //boost::shared_ptr< L3::Estimator::IterativeDescent<double> > algo( new L3::Estimator::IterativeDescent<double>( boost::shared_ptr< L3::Estimator::CostFunction<double> >( cost_function), experience->experience_pyramid ));
    //boost::shared_ptr< L3::Estimator::Algorithm<double> > algo( new L3::Estimator::Minimisation<double>( boost::shared_ptr< L3::Estimator::CostFunction<double> >(cost_function), experience->experience_pyramid ));
    //boost::shared_ptr< L3::Estimator::Algorithm<double> > algo( new L3::Estimator::PassThrough<double>(cost_function, experience->experience_pyramid ));
    //boost::shared_ptr< L3::Estimator::Algorithm<double> > algo( new L3::Estimator::Hybrid<double>( boost::shared_ptr< L3::Estimator::CostFunction<double> >(cost_function), experience->experience_pyramid ));
    
    //boost::shared_ptr< L3::Estimator::Algorithm<double> > algo( new L3::Estimator::BFGS<double>( boost::shared_ptr< L3::Estimator::CostFunction<double> >(cost_function), experience->experience_pyramid ));

    // Create runner
    boost::shared_ptr< L3::EstimatorRunner > runner( new L3::EstimatorRunner( dataset, mission, experience.get() ) );
    
    //boost::shared_ptr< L3::Estimator::Algorithm<double> > algo( new L3::Estimator::ParticleFilter<double>( boost::shared_ptr< L3::Estimator::CostFunction<double> >(cost_function), experience->experience_pyramid, runner->ics_velocity_provider) );
    boost::shared_ptr< L3::Estimator::Algorithm<double> > algo( new L3::Estimator::EKF<double>( boost::shared_ptr< L3::Estimator::CostFunction<double> >(cost_function), experience->experience_pyramid, runner->lhlv_velocity_provider ) );
    
    runner->setAlgorithm( algo );
    runner->start();

    boost::shared_ptr< L3::Container > container( new L3::Container() );

    container->dataset    = boost::shared_ptr<L3::Dataset>( dataset );
    container->mission    = boost::shared_ptr< L3::Configuration::Mission >( mission );
    container->experience = experience;
    container->runner     = runner;

    glv::Window win(1400, 800, "Visualisation::Estimation");

    L3::Visualisers::EstimatorLayout layout( win );
    
    L3::Interface* command_interface = new L3::CommandInterface( &layout, container );
    L3::Interface* lua_interface = new L3::LuaInterface();

    (*dynamic_cast< L3::Visualisers::GLVInterface* >( layout.scripting_interface.get() ) ) << command_interface << lua_interface;

    layout.load( runner.get(), experience );

    layout.run();

}

