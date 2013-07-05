#include "L3.h"

int main()
{
    boost::shared_ptr< L3::Dataset > dataset = boost::make_shared< L3::Dataset >( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" );

    dataset->validate();
    
    std::cout << *dataset << std::endl;

    L3::ReflectanceLoader loader( *dataset, 4 );

}


