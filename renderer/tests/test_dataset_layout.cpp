#include <iostream>
#include <fstream>

#include "Layouts.h"

int main()
{
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );

    if( !dataset.validate() && dataset.load() )
        exit(-1);

    glv::Window win(1400, 800, "Visualisation::Layout");

    L3::Visualisers::DatasetLayout layout(win);

    glv::GLV top;

    layout.load( &dataset );

    layout.run( top );

}
