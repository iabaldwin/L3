#include "L3.h"

int main()
{
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" );

    if(! dataset.validate() )
        throw std::exception(); 

    std::cout << dataset << std::endl;

    L3::ExperienceBuilder experience_builder( dataset, 100, 200 );
}
