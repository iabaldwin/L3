#include "L3.h"

int main()
{
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-08-09-36-42-WOODSTOCK-SLOW/" );
    L3::ExperienceLoader experience_loader( dataset );

    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;

    boost::timer t;

    for ( int i=0; i<100; i++ )
        try
        {
            // Random position update, this is the worst case
            experience->update( random()%100, random()%100 );
     
            boost::shared_ptr<L3::PointCloud<double> > cloud;


            t.restart();
            while( !experience->getExperienceCloud( cloud ))
                continue;

            std::cout << cloud->num_points << " in " << t.elapsed() << std::endl;
               
        }
        catch( std::exception& e )
        {
            std::cout << i << ":" << e.what() << std::endl;
            break;
        }
}
