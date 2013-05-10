#include <iostream>
#include <fstream>

#include "Layouts.h"

int main( int argc, char* argv[] )
{

    if ( argc != 2 ) 
    {
        std::cerr << "Usage: " << argv[0] << " <dataset>" << std::endl;
        exit(-1);
    }

    char* dataset_directory = argv[1];

    try
    {
        L3::Dataset dataset( dataset_directory );

        if( !(dataset.validate() && dataset.load() ) )
            exit(-1);
        
        L3::Configuration::Mission mission( dataset );

        glv::Window win(1400, 800, "Visualisation::DatasetLayout");

        L3::Visualisers::DatasetLayout layout(win);
        
        L3::DatasetRunner* runner = new L3::DatasetRunner( &dataset, &mission );

        runner->start();

        layout.load( runner );

        layout.run();

    }
    catch( L3::no_such_folder )
    {
        std::cout << dataset_directory <<  " is not a valid dataset" << std::endl;
        exit(-1);
    }

}
