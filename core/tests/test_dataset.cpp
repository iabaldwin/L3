#include "Dataset.h"


int main()
{
    try
    {

        L3::Dataset d( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );

        if( d.validate() )
            d.load();

        std::cout << d << std::endl;
    }
    catch( L3::no_such_folder& e  )
    {
        std::cout << "Dataset does not exist" << std::endl; 
    }
}
