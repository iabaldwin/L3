#include <iostream>
#include <iterator>
#include <fstream>

#include "L3.h"

int main( int argc, char* argv[] )
{
    if ( argc != 2 ) 
    {
        std::cerr << "Usage: " << argv[0] << " <dataset_list.list>" << std::endl;
        exit(-1);
    }


    std::vector< std::string > dataset_targets;

    std::ifstream dataset_list( argv[1], std::ios::in );

    std::copy( std::istream_iterator<std::string>( dataset_list ),
                std::istream_iterator<std::string>(),
                std::back_inserter( dataset_targets ) );

    std::copy( dataset_targets.begin(),
                dataset_targets.end(),
                std::ostream_iterator< std::string >( std::cout, "\n" ) );

    /*
     *  L3
     */
  
    for( std::vector<std::string>::iterator it = dataset_targets.begin();
            it != dataset_targets.end();
            it++ )
    {

        boost::shared_ptr< L3::Dataset > dataset;

        try
        {
            dataset = boost::make_shared< L3::Dataset > ( *it );
        }
        catch(...)
        {
            std::cerr << "No such dataset <" << *it << ">" << std::endl;
            continue;
        }

        if( !( dataset->validate() && dataset->load() ) )
        {
            std::cerr << "Could not validate <" << *it << ">" << std::endl;
            continue;
        }

        // Configuration
        L3::Configuration::Mission* mission = new L3::Configuration::Mission( *dataset ) ;

        // Experience
        L3::Dataset experience_dataset( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" );
        L3::ExperienceLoader experience_loader( experience_dataset );
        boost::shared_ptr<L3::Experience> experience = experience_loader.experience;

        // Estimator
        L3::Estimator::CostFunction<double>* cost_function = new L3::Estimator::MICostFunction<double>();

        // Create runner
        //boost::shared_ptr< L3::EstimatorRunner > runner( new L3::EstimatorRunner( dataset.get(), mission, experience.get() ) );
        boost::shared_ptr< L3::EstimatorRunner > runner( new L3::EstimatorRunner( dataset.get(), mission, experience.get(), 1.0 ) );
        
        runner->stand_alone = true;

        // Create algorithm
        boost::shared_ptr< L3::Estimator::Algorithm<double> > algo( new L3::Estimator::UKF<double>( boost::shared_ptr< L3::Estimator::CostFunction<double> >(cost_function), experience->experience_pyramid, runner->ics_velocity_provider ) );

        runner->setAlgorithm( algo );
        runner->start();

        while( true )
        {
            usleep( 10.0*1e6 );
        }
    }
}

