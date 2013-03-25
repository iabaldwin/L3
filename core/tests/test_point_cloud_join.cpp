#include "L3.h"

int main()
{

    // Load dataset
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-08-09-36-42-WOODSTOCK-SLOW/" );

    L3::ExperienceLoader experience_loader( dataset );

    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;

    experience->update( 150, 150);

    boost::shared_ptr<L3::PointCloud<double> > cloud;

    // Wait for synch()
    while( !experience->getExperienceCloud( cloud ) )
        continue;

    std::cout << *cloud << std::endl;
}
