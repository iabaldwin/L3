#include "L3.h"


int main( int argc, char* argv[] )
{
    // Load datset
  
    boost::shared_ptr< L3::Dataset > dataset;

    if ( argc == 1)
        dataset.reset( new L3::Dataset( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" ) );
    else
        dataset.reset( new L3::Dataset( argv[1] ));


    // Load experience
    L3::ExperienceLoader experience_loader( *dataset );

    boost::shared_ptr<L3::Experience> experience = experience_loader.experience;

    L3::Timing::SysTimer t;
    for ( int i=0; i<100; i-- )
    {
        experience->update( random()%100, random()%100 );
    
        t.begin();
        L3::ReadLock( experience->experience_pyramid->mutex );
        std::cout << (*experience->experience_pyramid)[0]->x_bins<< " x" 
            << (*experience->experience_pyramid)[0]->y_bins << " in " 
            << t.elapsed() << "s" << std::endl;
    
 
        double x = random()%100;
        double y = random()%100;
        double q = random()%100;

        experience->getClosestPose( L3::SE3( x, y, 0, 0, 0, q ) );
    }

}
