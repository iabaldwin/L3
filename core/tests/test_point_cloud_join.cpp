#include "L3.h"

int main()
{
    L3::ExperienceLoader experience_loader;

    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;

    experience->update( 150, 150);

    boost::shared_ptr<L3::PointCloud<double> > cloud;

    // Wait for synch()
    while( !experience->getExperienceCloud( cloud ) )
        continue;

    std::cout << *cloud << std::endl;
}
