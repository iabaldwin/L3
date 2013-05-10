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

    char* dataset_directory = argv[1];

    /*
     *  L3
     */


    for( int i=0; i<1000; i++ )
    { 
        L3::Dataset dataset( dataset_directory );

        if( !( dataset.validate() && dataset.load() ) )
            exit(-1);

        /*
         *Configuration
         */
        L3::Configuration::Mission mission( dataset );

        // Create runner
    
        boost::scoped_ptr< L3::EstimatorRunner > runner( new L3::EstimatorRunner( &dataset, &mission) );
        //L3::EstimatorRunner runner( &dataset, &mission);

        runner->start();

        std::stringstream ss;
        ss.precision( 16 );

        int target = 100000000;

            
        int counter = 0;
        while( true )
        {
            //ss << runner.current_time;

            //char* res = readline( (ss.str() + " >> ").c_str() ); 

            //if( std::string(res) == "stop" )
            //break;

            //if ( !res )
            //break;

            //ss.str( std::string("") );

            if ( counter++ > target )
            {
                std::cout << counter-1 << " of " << target<< std::endl;
                break;
            }
        }

    }
}
