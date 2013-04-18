#include <iostream>
#include <fstream>

#include "Layouts.h"

struct test_GLV : glv::GLV
{

    test_GLV()
    {

    }
     
    //void onAnimate(double dt, GLV& g)
    //{
        //std::cout << dt << std::endl;
    //}


};

int main( int argc, char* argv[] )
{
    //L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );

    char* dataset_directory = argv[1];

    try
    {
        L3::Dataset dataset( dataset_directory );

        if( !(dataset.validate() && dataset.load() ))
            exit(-1);
        glv::Window win(1400, 800, "Visualisation::Layout");

        L3::Visualisers::DatasetLayout layout(win);

        //glv::GLV top;
        test_GLV top;

        layout.load( &dataset );

        layout.run( top );

    }
    catch( L3::no_such_folder )
    {
        std::cout << dataset_directory <<  " is not a valid dataset" << std::endl;
        exit(-1);
    }

}
