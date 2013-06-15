#include <iostream>
#include <fstream>
#include <sstream>

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
    L3::DatasetRunner runner( &dataset, &mission);

    runner.start();

    std::stringstream ss;
    //std::cout.precision( 15 );
    ss.precision( 15 );

    int counter = 0;

    char glyphs[] = { '\\', '|', '/', '-' };

    while( true )
    {
        ss << ">>" << glyphs[counter++] << ":" << runner.current_time;
   
        if ( counter == sizeof(glyphs)/sizeof(char) )
            counter = 0;

        std::cout << ss.str();
        std::cout.flush();
        std::cout << "\r";
        ss.str("");

        usleep( .2 * 1e6 );
    
    }
}

