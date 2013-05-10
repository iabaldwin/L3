#include "Dataset.h"


int main()
{
    try
    {

        for( int i=0; i<100; i++ )
        {
            boost::shared_ptr< L3::Dataset > dataset(  new L3::Dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" ) );
            dataset->validate();
            dataset->load();
            std::cout << *dataset << std::endl;

            dataset.reset();
        }
    }
    catch( L3::no_such_folder& e  )
    {
        std::cout << "Dataset does not exist" << std::endl; 
    }
}
