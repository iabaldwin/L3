#include "Dataset.h"


int main()
{
    L3::Dataset d( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );

    if( d.validate() )
        d.load();
    else
        std::cerr << "Could not load " << d.path() << std::endl;

    std::cout << d << std::endl;
}
