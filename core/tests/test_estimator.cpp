#include <iostream>
#include <fstream>
#include <sstream>

#include <readline/readline.h>

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
    L3::Dataset dataset( dataset_directory );

    if( !( dataset.validate() && dataset.load() ) )
        exit(-1);
    
    /*
     *Configuration
     */
    L3::Configuration::Mission mission( dataset );

    // Create runner
    L3::EstimatorRunner runner( &dataset, &mission);
    //L3::DatasetRunner runner( &dataset, &mission);

    runner.start();

    std::stringstream ss;
  
    while( true )
    {
        ss << runner.current_time - dataset.start_time;
        
        char* res = readline( (ss.str() + " >> ").c_str() ); 

        if ( !res )
            break;
   
        ss.str( std::string("") );
    }
}

