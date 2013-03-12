#include "L3.h"


int main()
{
    // Load datset
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );

    if(! dataset.validate() )
        throw std::exception(); 
    std::cout << dataset << std::endl;

    // Build experience
    L3::ExperienceBuilder experience_builder( dataset, 100.0 );

    // Build experience
    L3::ExperienceLoader experience_loader;

    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;

    for ( int i=0; i<100; i++ )
        try
        {
        experience->load( i );
        experience->update( random()%100, random()%100 );
        }
        catch(...)
        {
        }


}
