#include <iostream>
#include <fstream>

#include "Layouts.h"

int main( int argc, char* argv[] )
{
    char* dataset_directory = argv[1];

    try
    {
        L3::Dataset dataset( dataset_directory );

        if( !(dataset.validate() && dataset.load() ) )
            exit(-1);
        
        glv::Window win(1400, 800, "Visualisation::DatasetLayout");

        L3::Visualisers::DatasetLayout layout(win, &dataset);

        glv::GLV top;

        layout.run( top );

    }
    catch( L3::no_such_folder )
    {
        std::cout << dataset_directory <<  " is not a valid dataset" << std::endl;
        exit(-1);
    }

}
