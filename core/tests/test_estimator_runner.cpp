#include <iostream>
#include <fstream>

#include "L3.h"

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
    
    // Create runner
    boost::shared_ptr< L3::EstimatorRunner > runner( new L3::EstimatorRunner( dataset, mission, experience.get() ) );
    runner->stand_alone = true;
    
    boost::shared_ptr< L3::Estimator::Algorithm<double> > algo( new L3::Estimator::ParticleFilter<double>( boost::shared_ptr< L3::Estimator::CostFunction<double> >(cost_function), experience->experience_pyramid, runner->lhlv_velocity_provider ) );
    
    runner->setAlgorithm( algo );
    runner->start();


    while( true )
    {
        usleep( 1.0*1e6 );
    }
}

