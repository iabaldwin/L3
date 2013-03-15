#include "L3.h"

int main()
{
    L3::ExperienceLoader experience_loader;

    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;

    for ( int i=0; i<100; i++ )
        try
        {
            experience->update( random()%100, random()%100 );
     
            boost::shared_ptr<L3::PointCloud<double> > cloud;

            while( !experience->getExperienceCloud( cloud ))
                continue;

            std::cout << cloud->num_points << std::endl;
               
        }
        catch( std::exception& e )
        {
            std::cout << i << ":" << e.what() << std::endl;
            break;
        }
}
