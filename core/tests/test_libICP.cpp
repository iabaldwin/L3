#include <iostream>
#include <fstream>
#include <sstream>

#include <readline/readline.h>
#include <boost/scoped_ptr.hpp>

#include "L3.h"

int main( int argc, char* argv[] )
{
    if ( argc != 2 ) 
    {
        std::cerr << "Usage: " << argv[0] << " <dataset>" << std::endl;
        exit(-1);
    }

    /*
     *  L3
     */
    char* dataset_target = argv[1];
        
    boost::shared_ptr< L3::Dataset > experience_dataset;
    experience_dataset.reset( new L3::Dataset( "/Users/ian/code/datasets/2012-02-08-09-36-42-WOODSTOCK-SLOW/" ) );

    // Load experience
    L3::ExperienceLoader experience_loader( *experience_dataset );

    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;

    L3::Dataset dataset( dataset_target );

    if( !( dataset.validate() && dataset.load() ) )
        exit(-1);

    /*
     *  Configuration
     */
    L3::Configuration::Mission mission( dataset );

    //L3::Estimator::CostFunction<double>* cost_function = new L3::Estimator::MICostFunction<double>();
    //boost::shared_ptr< L3::Estimator::IterativeDescent<double> > algo( new  L3::Estimator::IterativeDescent<double>(cost_function, experience->experience_pyramid ));

    // Create runner
    //boost::scoped_ptr< L3::EstimatorRunner > runner( new L3::EstimatorRunner( &dataset, &mission, experience.get() ) );
    boost::scoped_ptr< L3::DatasetRunner > runner( new L3::DatasetRunner( &dataset, &mission ) );

    //runner->setAlgorithm( algo );
    runner->start();

    std::stringstream ss;
    ss.precision( 16 );

    while( true )
        usleep( 1.0*1e6 );

}
