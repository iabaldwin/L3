#include "L3.h"


int main()
{
    //L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-08-09-36-42-WOODSTOCK-SLOW/" );

    if(! dataset.validate() )
        throw std::exception(); 

    std::cout << dataset << std::endl;

    L3::ExperienceBuilder experience_builder( dataset );
}
