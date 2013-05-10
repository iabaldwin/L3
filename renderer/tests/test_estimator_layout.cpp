#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "Layouts.h"
#include "Container.h"

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
    L3::Dataset dataset( dataset_directory );
   
    if( !( dataset.validate() && dataset.load() ) )
        exit(-1);

    // Configuration
    L3::Configuration::Mission mission( dataset );

    // Experience
    L3::Dataset experience_dataset( "/Users/ian/code/datasets/2012-02-08-09-36-42-WOODSTOCK-SLOW/" );
    L3::ExperienceLoader experience_loader( experience_dataset );
    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;

    // Estimator
    L3::Estimator::CostFunction<double>* kl_cost_function = new L3::Estimator::KLCostFunction<double>();
    L3::Estimator::IterativeDescent<double> algo( kl_cost_function, experience->experience_pyramid );
    
    // Create runner
    L3::EstimatorRunner runner( &dataset, &mission, experience.get() );

    runner.setAlgorithm( &algo ) 
            .start();

    //L3::Container container( ; 

    glv::Window win(1400, 800, "Visualisation::Estimator");

    L3::Visualisers::EstimatorLayout layout( win );
    
    layout.load( &runner, experience );

    layout.run();
}

