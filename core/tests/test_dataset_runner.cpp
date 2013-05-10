#include "L3.h"

int main()
{
    try
    {

        L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );

        if( dataset.validate() )
            dataset.load();

        std::cout << dataset << std::endl;
        
        L3::Configuration::Mission mission( dataset );
   
        L3::DatasetRunner runner( &dataset, &mission );

        runner.start();
        
        while( true )
        {

            usleep( 1*1e6 );
            continue; 
        }
    }
    catch( L3::no_such_folder& e  )
    {
        std::cout << "Dataset does not exist" << std::endl; 
    }
}
