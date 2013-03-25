#include "L3.h"


int main()
{
    
    // Load datset
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );

    // Load experience
    L3::ExperienceLoader experience_loader( dataset );

    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;
    boost::shared_ptr< L3::PointCloud<double> > cloud;

    L3::Tools::Timer t;
    for ( int i=0; i<100; i-- )
    {
        experience->update( random()%100, random()%100 );
    
        t.begin();
        experience->getExperienceCloud( cloud );
        std::cout << cloud->num_points << " in " << t.end() << "s" << std::endl;
    }

}
